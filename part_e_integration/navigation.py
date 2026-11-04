"""LiDAR obstacle map and a small A* waypoint planner.

Only estimated pose and measured ranges enter this module. Unknown space is
initially traversable; new scans can stop and replan a leg before an obstacle.
"""

import heapq

import numpy as np

import controller

CELL = 0.10
ROOM_HALF_SIZE = 3.0
CLEARANCE = 0.40  # robot footprint plus margin for odometry and sparse LiDAR beams
START_MARGIN = 0.15  # allow departure when noise puts the current cell inside the margin


class ObstacleMap:
    def __init__(self, lidar_position, lidar_angles):
        self.lidar_position = np.asarray(lidar_position)
        self.angles = np.asarray(lidar_angles)
        self.coordinates = np.arange(-ROOM_HALF_SIZE, ROOM_HALF_SIZE + CELL / 2, CELL)
        self.x, self.y = np.meshgrid(self.coordinates, self.coordinates, indexing="ij")
        # Room boundary is known; internal obstacles must come from LiDAR.
        self.blocked = (np.abs(self.x) > 2.7) | (np.abs(self.y) > 2.7)

    def cell(self, point):
        indices = np.rint((np.asarray(point) + ROOM_HALF_SIZE) / CELL).astype(int)
        return tuple(np.clip(indices, 0, len(self.coordinates) - 1))

    def point(self, cell):
        return self.coordinates[list(cell)]

    def update(self, pose, ranges, raw_ranges):
        c, s = np.cos(pose[2]), np.sin(pose[2])
        origin = pose[:2] + np.array([[c, -s], [s, c]]) @ self.lidar_position
        # Keep the simulator's no-return mask; B replaces -1 with maximum range.
        valid = np.isfinite(raw_ranges) & (raw_ranges >= 0.12) & (raw_ranges < 3.5)
        angles = self.angles[valid] + pose[2]
        hits = origin + ranges[valid, None] * np.c_[np.cos(angles), np.sin(angles)]
        for x, y in hits:
            self.blocked |= (self.x - x)**2 + (self.y - y)**2 <= CLEARANCE**2

    def line_free(self, start, end, allow_start=False):
        count = max(2, int(np.linalg.norm(np.asarray(end) - start) / (CELL / 2)) + 1)
        for point in np.linspace(start, end, count):
            if allow_start and np.linalg.norm(point - start) <= START_MARGIN:
                continue
            if self.blocked[self.cell(point)]:
                return False
        return True

    def plan(self, start, goal):
        """Return a short list of waypoints, or None if the observed map blocks it."""
        first, last = self.cell(start), self.cell(goal)
        blocked = self.blocked.copy()
        # Inflation is extra clearance, not a measured wall. Let the robot leave
        # a small part of that margin around its own centre instead of deadlocking.
        blocked[(self.x - start[0])**2 + (self.y - start[1])**2 <= START_MARGIN**2] = False
        if blocked[last]:
            return None
        frontier = [(0.0, first)]
        cost, parent = {first: 0.0}, {}
        moves = [(i, j) for i in (-1, 0, 1) for j in (-1, 0, 1) if i or j]
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == last:
                break
            for dx, dy in moves:
                neighbour = current[0] + dx, current[1] + dy
                if not all(0 <= value < len(self.coordinates) for value in neighbour):
                    continue
                if blocked[neighbour]:
                    continue
                # Do not cut diagonally between touching occupied cells.
                if dx and dy and (blocked[current[0] + dx, current[1]]
                                  or blocked[current[0], current[1] + dy]):
                    continue
                new_cost = cost[current] + np.hypot(dx, dy)
                if new_cost >= cost.get(neighbour, float("inf")):
                    continue
                cost[neighbour], parent[neighbour] = new_cost, current
                priority = new_cost + np.linalg.norm(np.array(neighbour) - last)
                heapq.heappush(frontier, (priority, neighbour))
        if last not in cost:
            return None
        cells = [last]
        while cells[-1] != first:
            cells.append(parent[cells[-1]])
        points = [np.asarray(start), *[self.point(cell) for cell in cells[::-1][1:-1]], np.asarray(goal)]
        # Greedily skip grid corners where the straight segment is clear.
        route = [points[0]]
        index = 0
        while index < len(points) - 1:
            next_index = index + 1
            for candidate in range(len(points) - 1, index, -1):
                if self.line_free(points[index], points[candidate], allow_start=index == 0):
                    next_index = candidate
                    break
            route.append(points[next_index])
            index = next_index
        return route


class Navigator:
    def __init__(self, marker, normal, lidar_position, lidar_angles):
        self.map = ObstacleMap(lidar_position, lidar_angles)
        self.pre_dock, self.dock, _ = controller.dock_targets(marker, normal)
        self.tracker = None
        self.target = None
        self.mode = "planning"
        self.replans = 0

    def step(self, estimate, t, scan=None, raw_scan=None):
        if self.mode in ("docked", "timeout"):
            return 0.0, 0.0
        if scan is not None:
            self.map.update(estimate, scan, raw_scan)
            if self.mode == "navigate" and not self.map.line_free(estimate[:2], self.target, allow_start=True):
                self.tracker = None
                self.mode = "planning"

        if self.mode == "planning":
            if np.linalg.norm(estimate[:2] - self.pre_dock) < 0.06:
                reference = controller.Reference([self.pre_dock, self.dock])
                self.mode = "dock"
            else:
                route = self.map.plan(estimate[:2], self.pre_dock)
                if route is None:
                    return 0.0, 0.0
                self.target = route[1]
                reference = controller.Reference([estimate[:2], self.target], final_straight=0.0)
                self.mode = "navigate"
                self.replans += 1
            self.tracker = controller.LQRTracker(reference)

        v, w = self.tracker.step(estimate, t)
        if self.tracker.mode == "timeout":
            self.mode = "timeout"
        elif self.tracker.docked:
            self.mode = "docked" if self.mode == "dock" else "planning"
        return v, w
