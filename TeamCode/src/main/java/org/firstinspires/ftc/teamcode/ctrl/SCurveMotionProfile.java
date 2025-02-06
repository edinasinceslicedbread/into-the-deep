package org.firstinspires.ftc.teamcode.ctrl;
public class SCurveMotionProfile {
    public static class Profile {
        public double position;
        public double velocity;
        public double acceleration;
        public double time;
        public Profile(double position, double velocity, double acceleration, double time) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.time = time;
        }
        public int getPositionTicks() {
            return (int) Math.round(this.position);
        }
        public int getVelocityTicks() {
            return (int) Math.round(this.velocity);
        }
        public int getAccelerationTicks() {
            return (int) Math.round(this.acceleration);
        }
    }
    public static Profile[] generateProfile(double startPosition, double endPosition, double maxVelocity, double maxAcceleration, double maxJerk) {
        // Calculate the total distance
        double totalDistance = endPosition - startPosition;
        // Determine the direction of motion
        double direction = Math.signum(totalDistance); // +1 for positive direction, -1 for negative direction
        // Use absolute values for calculations
        double absTotalDistance = Math.abs(totalDistance);
        double absMaxVelocity = Math.abs(maxVelocity);
        double absMaxAcceleration = Math.abs(maxAcceleration);
        double absMaxJerk = Math.abs(maxJerk);
        // Calculate the time to reach max acceleration
        double timeToMaxAcceleration = absMaxAcceleration / absMaxJerk;
        // Calculate the distance covered during the jerk phases
        double distanceToMaxAcceleration = (1.0 / 6.0) * absMaxJerk * Math.pow(timeToMaxAcceleration, 3);
        // Calculate the time to reach max velocity
        double timeToMaxVelocity = (absMaxVelocity - absMaxAcceleration * timeToMaxAcceleration) / absMaxAcceleration;
        // Calculate the total distance to reach max velocity
        double distanceToMaxVelocity = 2 * distanceToMaxAcceleration + absMaxAcceleration * timeToMaxVelocity;
        // Determine if we reach max velocity or not
        double totalTime;
        if (distanceToMaxVelocity * 2 >= absTotalDistance) {
            // Triangular S-curve profile (we don't reach max velocity)
            timeToMaxVelocity = 0;
            timeToMaxAcceleration = Math.cbrt(absTotalDistance / (4 * absMaxJerk));
            totalTime = 4 * timeToMaxAcceleration;
        } else {
            // Trapezoidal S-curve profile (we reach max velocity)
            double distanceAtMaxVelocity = absTotalDistance - 2 * distanceToMaxVelocity;
            double timeAtMaxVelocity = distanceAtMaxVelocity / absMaxVelocity;
            totalTime = 4 * timeToMaxAcceleration + 2 * timeToMaxVelocity + timeAtMaxVelocity;
        }
        // Create the profile
        int arraySize = (int) Math.ceil(totalTime / 0.01) + 1;
        Profile[] profile = new Profile[arraySize];
        double currentTime = 0.0;
        double cumulativePosition = startPosition; // Track cumulative position
        double currentVelocity = 0.0; // Start with zero velocity
        double currentAcceleration = 0.0; // Start with zero acceleration
        int i = 0;
        while (i < arraySize - 1) {
            if (currentTime < timeToMaxAcceleration) {
                // Positive jerk phase
                currentAcceleration = direction * absMaxJerk * currentTime;
                currentVelocity += currentAcceleration * 0.01;
                cumulativePosition += currentVelocity * 0.01;
            } else if (currentTime < 2 * timeToMaxAcceleration) {
                // Constant acceleration phase
                currentAcceleration = direction * absMaxAcceleration;
                currentVelocity += currentAcceleration * 0.01;
                cumulativePosition += currentVelocity * 0.01;
            } else if (currentTime < 3 * timeToMaxAcceleration) {
                // Negative jerk phase
                double t = currentTime - 2 * timeToMaxAcceleration;
                currentAcceleration = direction * (absMaxAcceleration - absMaxJerk * t);
                currentVelocity += currentAcceleration * 0.01;
                cumulativePosition += currentVelocity * 0.01;
            } else if (currentTime < totalTime - 3 * timeToMaxAcceleration) {
                // Constant velocity phase
                currentAcceleration = 0;
                currentVelocity = direction * absMaxVelocity;
                cumulativePosition += currentVelocity * 0.01;
            } else {
                // Deceleration phases (reverse of acceleration phases)
                double t = totalTime - currentTime;
                currentAcceleration = -direction * absMaxJerk * t;
                currentVelocity += currentAcceleration * 0.01;
                cumulativePosition += currentVelocity * 0.01;
            }
            // Add the profile point
            profile[i] = new Profile(cumulativePosition, currentVelocity, currentAcceleration, currentTime);
            // Increment time and index
            currentTime += 0.01;
            i++;
        }
        // Adjust the final point to ensure it ends at (endPosition, 0, 0)
        profile[i] = new Profile(endPosition, 0, 0, totalTime);
        return profile;
    }
}
