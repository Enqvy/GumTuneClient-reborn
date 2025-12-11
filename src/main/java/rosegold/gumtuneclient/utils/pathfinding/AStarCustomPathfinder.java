package rosegold.gumtuneclient.utils.pathfinding;

import net.minecraft.block.Block;
import net.minecraft.block.state.IBlockState;
import net.minecraft.init.Blocks;
import net.minecraft.util.BlockPos;
import net.minecraft.util.Vec3;
import rosegold.gumtuneclient.GumTuneClient;
import rosegold.gumtuneclient.modules.player.PathFinding;
import rosegold.gumtuneclient.utils.ModUtils;
import rosegold.gumtuneclient.utils.RaytracingUtils;
import rosegold.gumtuneclient.utils.VectorUtils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.PriorityQueue;

public class AStarCustomPathfinder {
    private final Vec3 startVec3;
    private final Vec3 endVec3;
    private ArrayList<Vec3> path = new ArrayList<>();
    // Optimization: Use HashMap for o(1) lookups instead of looping through lists
    private final HashMap<BlockPos, Hub> hubs = new HashMap<>();
    // Optimization: Use PriorityQueue for o(1) sorting
    private final PriorityQueue<Hub> hubsToWork = new PriorityQueue<>(new CompareHub());
    private final double minDistanceSquared;
    public static long counter = 0;

    private static final Vec3[] flatCardinalDirections = {
            new Vec3(1, 0, 0),
            new Vec3(-1, 0, 0),
            new Vec3(0, 0, 1),
            new Vec3(0, 0, -1)
    };

    public AStarCustomPathfinder(Vec3 startVec3, Vec3 endVec3, double minDistanceSquared) {
        this.startVec3 = VectorUtils.floorVec(startVec3);
        this.endVec3 = VectorUtils.floorVec(endVec3);
        this.minDistanceSquared = minDistanceSquared;
    }

    public ArrayList<Vec3> getPath() {
        return path;
    }

    public void compute() {
        compute(2000, 1);
    }

    public void compute(int loops, int depth) {
        long startTime = System.currentTimeMillis();
        counter = 0;
        PathFinding.renderHubs.clear();
        path.clear();
        hubsToWork.clear();
        hubs.clear();

        Hub startHub = new Hub(startVec3, null, startVec3.distanceTo(endVec3), 0.0, 0.0);
        hubsToWork.add(startHub);
        hubs.put(new BlockPos(startVec3), startHub);

        search:
        while (!hubsToWork.isEmpty()) {
            // Safety break
            if (System.currentTimeMillis() - startTime > 1000) break;

            Hub hub = hubsToWork.poll();
            
            PathFinding.renderHubs.add(new BlockPos(VectorUtils.ceilVec(hub.getLoc())));

            if (checkGoal(hub.getLoc())) {
                path = hub.getPath();
                break search;
            }

            for (BlockPos blockPos : RaytracingUtils.getAllTeleportableBlocksNew(VectorUtils.ceilVec(hub.getLoc()).addVector(0.5, 1.62 - 0.08 + 1, 0.5), 16)) {
                Vec3 loc = new Vec3(blockPos);
                // Calculate actual distance cost for A*
                double cost = hub.getLoc().distanceTo(loc);
                if (addHub(hub, loc, cost)) {
                    break search;
                }
            }
        }
        
        counter = System.currentTimeMillis() - startTime;
        ModUtils.sendMessage("Done calculating path, searched " + hubs.size() + " blocks, took: " + counter + "ms");
    }

    // Helper for checking goal
    private boolean checkGoal(Vec3 loc) {
         if (minDistanceSquared != 0.0 && loc.squareDistanceTo(endVec3) <= minDistanceSquared) return true;
         return (int)loc.xCoord == (int)endVec3.xCoord && 
                (int)loc.yCoord == (int)endVec3.yCoord && 
                (int)loc.zCoord == (int)endVec3.zCoord;
    }

    public static boolean checkPositionValidity(Vec3 loc) {
        return checkPositionValidity(new BlockPos((int) loc.xCoord, (int) loc.yCoord, (int) loc.zCoord));
    }

    public static boolean checkPositionValidity(BlockPos blockPos) {
        return canTeleportTo(blockPos);
    }

    private static boolean canVecBeSeen(Vec3 from, Vec3 to) {
        return RaytracingUtils.canVecBeSeenFromVec(from.addVector(0.5, 0.5, 0.5), to.addVector(0.5, 0.5, 0.5), 0.1f);
    }

    private static boolean canTeleportTo(BlockPos blockPos) {
        IBlockState blockState = GumTuneClient.mc.theWorld.getBlockState(blockPos);
        Block block = blockState.getBlock();
        return block.isCollidable() && block != Blocks.carpet && block != Blocks.skull &&
                block.getCollisionBoundingBox(GumTuneClient.mc.theWorld, blockPos, blockState) != null &&
                block != Blocks.wall_sign && block != Blocks.standing_sign &&
                GumTuneClient.mc.theWorld.getBlockState(blockPos.add(0, 1, 0)).getBlock() == Blocks.air &&
                GumTuneClient.mc.theWorld.getBlockState(blockPos.add(0, 2, 0)).getBlock() == Blocks.air;
    }

    public Hub isHubExisting(Vec3 loc) {
        return hubs.get(new BlockPos(loc));
    }

    public boolean addHub(Hub parent, Vec3 loc, double cost) {
        Hub existingHub = isHubExisting(loc);
        double totalCost = cost;
        if (parent != null) {
            totalCost += parent.getTotalCost(); // Accumulate G-Cost
        }
        
        if (checkGoal(loc)) {
            Hub goal = new Hub(loc, parent, 0, cost, totalCost);
            path = goal.getPath();
            return true;
        }

        if (existingHub == null) {
            // H-Cost (Heuristic) is distance to end
            double distToTarget = loc.distanceTo(endVec3);
            Hub newHub = new Hub(loc, parent, distToTarget, cost, totalCost);
            hubs.put(new BlockPos(loc), newHub);
            hubsToWork.add(newHub);
        } else if (totalCost < existingHub.getTotalCost()) {
            // Found a shorter path to an existing node
            existingHub.setLoc(loc);
            existingHub.setParent(parent);
            existingHub.setCost(cost);
            existingHub.setTotalCost(totalCost);
            
            // Re-sort priority queue
            hubsToWork.remove(existingHub);
            hubsToWork.add(existingHub);
        }
        return false;
    }

    private static class Hub {
        private Vec3 loc;
        private Hub parent;
        // Optimization: Path list removed to save RAM. Reconstructed via parents.
        private double squareDistanceToFromTarget; // H-Cost
        private double cost;
        private double totalCost; // G-Cost

        public Hub(Vec3 loc, Hub parent, double squareDistanceToFromTarget, double cost, double totalCost) {
            this.loc = loc;
            this.parent = parent;
            this.squareDistanceToFromTarget = squareDistanceToFromTarget;
            this.cost = cost;
            this.totalCost = totalCost;
        }

        public Vec3 getLoc() {
            return loc;
        }

        public Hub getParent() {
            return parent;
        }

        // Optimization: Reconstructs path backwards from parents
        public ArrayList<Vec3> getPath() {
            ArrayList<Vec3> path = new ArrayList<>();
            Hub current = this;
            while(current != null) {
                path.add(current.loc);
                current = current.parent;
            }
            Collections.reverse(path);
            return path;
        }

        public double getSquareDistanceToFromTarget() {
            return squareDistanceToFromTarget;
        }

        public double getCost() {
            return cost;
        }

        public void setLoc(Vec3 loc) {
            this.loc = loc;
        }

        public void setParent(Hub parent) {
            this.parent = parent;
        }

        public void setSquareDistanceToFromTarget(double squareDistanceToFromTarget) {
            this.squareDistanceToFromTarget = squareDistanceToFromTarget;
        }

        public void setCost(double cost) {
            this.cost = cost;
        }

        public double getTotalCost() {
            return totalCost;
        }

        public void setTotalCost(double totalCost) {
            this.totalCost = totalCost;
        }
    }

    public static class CompareHub implements Comparator<Hub> {
        @Override
        public int compare(Hub o1, Hub o2) {
            // F-Cost = G (TotalCost) + H (DistanceToTarget)
            return Double.compare(
                o1.getTotalCost() + o1.getSquareDistanceToFromTarget(), 
                o2.getTotalCost() + o2.getSquareDistanceToFromTarget()
            );
        }
    }
}