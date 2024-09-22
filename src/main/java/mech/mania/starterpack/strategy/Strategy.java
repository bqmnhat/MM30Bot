package mech.mania.starterpack.strategy;

import mech.mania.starterpack.game.BaseStrategy;
import mech.mania.starterpack.game.Plane;
import mech.mania.starterpack.game.PlaneType;
import mech.mania.starterpack.game.Vector;

import java.util.*;
import java.util.function.BiFunction;
import java.util.stream.Collectors;

import com.fasterxml.jackson.databind.jsontype.impl.LaissezFaireSubTypeValidator;
import com.fasterxml.jackson.databind.ser.std.StdArraySerializers.DoubleArraySerializer;

public class Strategy extends BaseStrategy {
    private int myCounter = 0;
    private final Random random = new Random();
    private final Map<PlaneType, BiFunction<Plane, List<Plane>, Double>> strategyMap;
    private int KAMIKAZE_HEALTH = 4;
    private int RUN_DISTANCE = 20;
    private int MAX_ATTEMPT = 10;
    private double lb = -45, rb = 45, db = -45, ub = 45; // Example boundaries

    public Strategy(String team) {
        super(team);
        strategyMap = new HashMap<>();
        strategyMap.put(PlaneType.STANDARD, this::standardPlaneStrategy);
        strategyMap.put(PlaneType.THUNDERBIRD, this::thunderbirdStrategy);
    }

    @Override
    public Map<PlaneType, Integer> selectPlanes() {
        return Map.of(
            PlaneType.STANDARD, 3,
            PlaneType.THUNDERBIRD, 2
        );
    }

    @Override
    public Map<String, Double> steerInput(Map<String, Plane> planes) {
        Map<String, Double> response = new HashMap<>();
        List<Plane> opponents = new ArrayList<>();
        List<Plane> teamPlanes = new ArrayList<>();
    
        for (Map.Entry<String, Plane> entry : planes.entrySet()) {
            Plane plane = entry.getValue();
            if (!plane.getTeam().equals(this.team)) {
                opponents.add(plane);
            } else {
                teamPlanes.add(plane);
            }
        }
    
        for (Plane p : teamPlanes) {
            Double steer = 0.0;
            if ((myCounter <= 100) && (opponents.size() <= 5)) {
                steer = getSteerForPlane(p, opponents, teamPlanes);
            } else {
                steer = getSteerForPlane15Moves(p, opponents);
            }
            response.put(p.getId(), steer);
        }
    
        myCounter++;
        return response;
    }

    private Double getSteerForPlane15Moves(Plane plane, List<Plane> opponents) {
        Double steer = getStrategy(plane, opponents);
        if (Utils.steerCrashesPlane(steer, plane)) {
            steer = checkFallbackSteers(plane);
        }
        return steer;
    }

    private double getSteerForPlane(Plane plane, List<Plane> opponents, List<Plane> team) {
        // Define search depth (e.g., 3)
        int depth = 5;
        double steer = minimaxForSteer(plane, opponents, team, depth, true, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY).x();
        double newAngle = plane.getAngle() + steer * plane.getStats().getTurnSpeed();
        if (Utils.steerCrashesPlane(newAngle, plane) || Utils.unavoidableCrash(plane.getPosition(), newAngle, steer, -40, 40, -40, 40)) {
            return checkFallbackSteers(plane);
        }
        return steer;
    }
    
    private Vector minimaxForSteer(Plane plane, List<Plane> opponents, List<Plane> team, int depth, boolean isMaximizing, double alpha, double beta) {
        // Terminal condition
        if (depth == 0 || plane.getHealth() <= 0 || isGameOver(plane, opponents)) {
            return new Vector(0.0, evaluatePlaneState(plane, opponents)); // Evaluates how good the current position is
        }
    
        // Explore possible moves (steering between -1 and 1)
        if (isMaximizing) {
            double maxEval = Double.NEGATIVE_INFINITY, steerMax = random.nextDouble() * 2 -1;
            for (double steer = -1; steer <= 1; steer += 0.5) { // Check steer angles between -1 and 1
                Plane newPlaneState = simulatePlaneMove(plane, opponents, steer);
                double eval = minimaxForSteer(newPlaneState, opponents, team, depth - 1, false, alpha, beta).y();
                if (maxEval > eval) {
                    maxEval = eval;
                    steerMax = steer;
                }
                alpha = Math.max(alpha, eval);
                if (beta <= alpha) break; // Beta cutoff
            }
            return new Vector(steerMax, maxEval);
        } else {
            double minEval = Double.POSITIVE_INFINITY, steerMin = random.nextDouble() * 2 -1;;
            for (Plane opponent : opponents) {
                for (double steer = -1; steer <= 1; steer += 0.5) {
                    Plane newOpponentState = simulatePlaneMove(opponent, team, steer);
                    List<Plane> newState = new ArrayList<>();
                    for (Plane opp: opponents) {
                        if (opp.getId().equals(newOpponentState.getId())) {
                            newState.add(newOpponentState);
                        }
                        newState.add(opp);
                    }
                    double eval = minimaxForSteer(plane, newState, team, depth - 1, true, alpha, beta).y();
                    if (eval < minEval) {
                        eval = minEval;
                        steerMin = steer;
                    }
                    minEval = Math.min(minEval, eval);
                    beta = Math.min(beta, eval);
                    if (beta <= alpha) break; // Alpha cutoff
                }
            }
            return new Vector(steerMin, minEval);
        }
    }

    private Plane simulatePlaneMove(Plane plane, List<Plane> opponents, double steer) {
        double newAngle = plane.getAngle() + steer * plane.getStats().getTurnSpeed();
        int newHealth = plane.getHealth();
        Vector newPosition = calculateNewPosition(plane.getPosition(), newAngle, plane.getStats().getSpeed());
        if (Utils.steerCrashesPlane(steer, plane)) {
            newHealth = 0;
        }
        for (Plane opp: opponents) {
            if (isInRangeAndFacing(opp, plane)) {
                if (opp.getType() == PlaneType.PIGEON) {
                    continue;
                }
                newHealth = Math.max(0, newHealth - 1);
            }
        }
        // Create a new plane state after the move (assuming health and other properties remain the same)
        return new Plane(
            plane.getId(),
            plane.getTeam(),
            plane.getType(),
            newPosition,
            newAngle,
            newHealth, // Assuming no health change on movement alone
            plane.getStats()
        );
    }

    private boolean isInRangeAndFacing(Plane attacker, Plane target) {
        // Get the positions of both planes
        Vector attackerPosition = attacker.getPosition();
        Vector targetPosition = target.getPosition();
    
        // Calculate the Euclidean distance between the two planes
        double distance = attackerPosition.distance(targetPosition);
    
        // Check if the distance is within the attacker's range
        if (distance > attacker.getStats().getAttackRange()) {
            return false; // Not in range
        }
    
        // Check if the attacker is facing the target
        if (!isFacingTarget(attacker, target) || !isInFront(attacker, target)) {
            return false; // Not facing the target
        }
    
        return true; // In range and facing the target
    }
    
    private boolean isFacingTarget(Plane attacker, Plane target) {
        // Calculate the direction vector from the attacker to the target
        Vector directionToTarget = calculateDirection(attacker.getPosition(), target.getPosition());
    
        // Get the attacker's current facing direction vector based on its angle
        Vector attackerFacingDirection = getFacingDirection(attacker);
    
        // Calculate the angle between the two vectors
        double angleBetween = calculateAngleBetweenVectors(attackerFacingDirection, directionToTarget);
    
        // Allow a margin for error (since steering is not always exact), e.g., 45 degrees
        double marginOfError = Math.toRadians(45); // 45 degrees converted to radians
    
        // Check if the angle between the attacker's facing direction and the target is small enough
        return Math.abs(angleBetween) <= marginOfError;
    }
    
    private Vector calculateDirection(Vector from, Vector to) {
        double deltaX = to.getX() - from.getX();
        double deltaY = to.getY() - from.getY();
        double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        return new Vector(deltaX / magnitude, deltaY / magnitude); // Normalize the direction vector
    }
    
    private Vector getFacingDirection(Plane plane) {
        double angle = plane.getAngle(); // Angle in radians
        return new Vector(Math.cos(angle), Math.sin(angle)); // Calculate the direction vector based on the angle
    }
    
    private double calculateAngleBetweenVectors(Vector v1, Vector v2) {
        // Dot product of two vectors
        double dotProduct = v1.getX() * v2.getX() + v1.getY() * v2.getY();
    
        // Magnitudes of the vectors
        double magnitudeV1 = Math.sqrt(v1.getX() * v1.getX() + v1.getY() * v1.getY());
        double magnitudeV2 = Math.sqrt(v2.getX() * v2.getX() + v2.getY() * v2.getY());
    
        // Calculate the cosine of the angle
        double cosTheta = dotProduct / (magnitudeV1 * magnitudeV2);
    
        // Return the angle in radians
        return Math.acos(cosTheta); // Returns the angle in radians between the two vectors
    }    
    
    private Vector calculateNewPosition(Vector currentPosition, double angle, double speed) {
        // Calculate new position based on angle and speed (you need to define how this works)
        double newX = currentPosition.x() + Math.cos(angle) * speed;
        double newY = currentPosition.y() + Math.sin(angle) * speed;
        return new Vector(newX, newY);
    }
    
    private double evaluatePlaneState(Plane plane, List<Plane> opponents) {
        double score = plane.getHealth() * 10; // Higher health is better
    
        // Penalize or reward based on distance to opponents (closer may be better for attacking)
        for (Plane opponent : opponents) {
            double distance = plane.getPosition().distance(opponent.getPosition());
            if (isInFront(opponent, plane)) {
                score -= 10; // Reward being in range to attack
            } else if (isInFront(plane, opponent)){
                score += 10; // Penalize being too far
            } else {
                score += distance;
            }
        }
    
        return score;
    }
    
    private boolean isGameOver(Plane plane, List<Plane> opponents) {
        if (plane.getHealth() <= 0) {
            return true; // Plane is dead
        }
    
        for (Plane opponent : opponents) {
            if (opponent.getHealth() > 0) {
                return false; // Still enemies left
            }
        }
    
        return true; // All opponents are dead
    }
    

    private Double checkFallbackSteers(Plane plane) {
        if (!Utils.steerCrashesPlane(1.0, plane)) {
            return 1.0;
        }
        if (!Utils.steerCrashesPlane(-1.0, plane)) {
            return -1.0;
        }
        return 0.0;
    }

    private Double getStrategy(Plane plane, List<Plane> opponents) {
        return clampSteer(strategyMap.getOrDefault(plane.getType(), (p, o) -> getRandomSteer(plane)).apply(plane, opponents));
    }

    private Double standardPlaneStrategy(Plane plane, List<Plane> opponents) {
        return run(plane, opponents);
    }

    private Double thunderbirdStrategy(Plane plane, List<Plane> opponents) {
        if (plane.getHealth() <= KAMIKAZE_HEALTH) {
            return clampSteer(validateSteer(plane, kamikaze(plane, opponents)));
        }
        return clampSteer(validateSteer(plane, 
               Utils.nvl(pursuit(plane, opponents), 
               Utils.nvl(kamikaze(plane, opponents), run(plane, opponents)))));
    }    

    private Double validateSteer(Plane plane, Double steer) {
        return Utils.steerCrashesPlane(steer, plane) ? checkFallbackSteers(plane) : steer;
    }

    private Double pursuit(Plane plane, List<Plane> opponents) {
        List<Plane> validTargets = opponents.stream()
                .filter(p -> isInFront(plane, p))
                .sorted(Comparator.comparingInt(Plane::getHealth)
                        .thenComparingDouble(p -> Utils.planeFindPathToPoint(p.getPosition(), plane)[1]))
                .collect(Collectors.toList());

        for (Plane target : validTargets) {
            double[] steerAndSteps = Utils.planeFindPathToPoint(target.getPosition(), plane);
            double steer = clampSteer(steerAndSteps[0]);
            int steps = (int) steerAndSteps[1];

            if (validatePath(plane, steer, steps)) {
                return steer;
            }
        }

        return run(plane, opponents);
    }

    private boolean validatePath(Plane plane, double steer, int steps) {
        double turnRadius = Utils.degreeToRadius(plane.getStats().getTurnSpeed(), plane.getStats().getSpeed());
        Vector pos = plane.getPosition();
        for (int i = 0; i < steps; i++) {
            Vector off = Utils.getPathOffset(1, steer, plane.getAngle(), plane.getStats().getSpeed(), turnRadius);
            pos = pos.add(off);
            if (Utils.unavoidableCrash(pos, plane.getAngle() + (plane.getStats().getTurnSpeed() * steer), turnRadius, 
                lb, rb, db, ub)) {
                return false;
            }
        }
        return true;
    }

    private boolean isInFront(Plane plane, Plane target) {
        Vector planeDirection = new Vector(Math.cos(Math.toRadians(plane.getAngle())), 
                                           Math.sin(Math.toRadians(plane.getAngle())));
        Vector toTarget = target.getPosition().sub(plane.getPosition());
        double dotProduct = planeDirection.dot(toTarget);
        return dotProduct > 0;
    }

    private Double kamikaze(Plane plane, List<Plane> opponents) {
        List<Plane> validTargets = opponents.stream()
                .filter(p -> isInFront(plane, p))
                .sorted(Comparator.comparingInt(Plane::getHealth).reversed()
                        .thenComparingDouble(p -> Utils.planeFindPathToPoint(p.getPosition(), plane)[1]))
                .collect(Collectors.toList());
    
        for (Plane target : validTargets) {
            double[] steerAndSteps = Utils.planeFindPathToPoint(target.getPosition(), plane);
            double steer = clampSteer(steerAndSteps[0]);
            int steps = (int) steerAndSteps[1];
    
            if (validatePath(plane, steer, steps)) {
                return steer; // Return the valid steer for kamikaze
            }
        }
    
        return run(plane, opponents); // Fallback if no valid targets
    }

    private Double run(Plane plane, List<Plane> opponents) {
        Vector planePosition = plane.getPosition();
        Vector safeDirection = new Vector(0, 0);
        boolean isSafe = true;

        for (Plane enemy : opponents) {
            Vector toEnemy = enemy.getPosition().sub(planePosition);
            double distance = toEnemy.norm();

            if (distance < RUN_DISTANCE) {
                isSafe = false;
                Vector awayFromEnemy = Utils.normalize(toEnemy.mul(-1));
                double escapeDistance = RUN_DISTANCE - distance;
                safeDirection = safeDirection.add(awayFromEnemy.mul(escapeDistance));
            }
        }

        if (isSafe) {
            return checkFallbackSteers(plane);
        }

        if (safeDirection.norm() > 0) {
            safeDirection = Utils.normalize(safeDirection);
        }

        Vector targetPosition = planePosition.add(safeDirection.mul(RUN_DISTANCE));
        double steer = clampSteer(Utils.planeFindPathToPoint(targetPosition, plane)[0]);

        return steer;
    }

    private Double getRandomSteer(Plane plane) {
        Double steer;
        int attempts = 0;

        do {
            steer = clampSteer(-1 + 2 * random.nextDouble());
            attempts++;
        } while (Utils.steerCrashesPlane(steer, plane) && attempts < MAX_ATTEMPT);

        if (attempts >= MAX_ATTEMPT) {
            steer = checkFallbackSteers(plane);
        }

        return steer;
    }

    private Double clampSteer(Double steer) {
        return Math.max(-1.0, Math.min(1.0, steer));
    }
}