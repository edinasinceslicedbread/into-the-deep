package org.firstinspires.ftc.teamcode.ctrl;
public class TrapezoidalMotionProfile {
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

    public static Profile[] generateProfile(double startPosition, double endPosition, double maxVelocity, double maxAcceleration) {
        // Calculate the total distance
        double totalDistance = endPosition - startPosition;
        // Determine the direction of motion
        double direction = Math.signum(totalDistance); // +1 for positive direction, -1 for negative direction
        // Use absolute values for calculations
        double absTotalDistance = Math.abs(totalDistance);
        double absMaxVelocity = Math.abs(maxVelocity);
        double absMaxAcceleration = Math.abs(maxAcceleration);
        // Calculate the time it takes to reach max velocity
        double timeToMaxVelocity = absMaxVelocity / absMaxAcceleration;
        // Calculate the distance traveled during acceleration
        double distanceToMaxVelocity = 0.5 * absMaxAcceleration * timeToMaxVelocity * timeToMaxVelocity;
        // Calculate the total time and distance
        double totalTime;
        if (distanceToMaxVelocity * 2 >= absTotalDistance) {
            // We don't reach max velocity, so we have a triangular profile
            timeToMaxVelocity = Math.sqrt(absTotalDistance / absMaxAcceleration);
            totalTime = timeToMaxVelocity * 2;
            distanceToMaxVelocity = absTotalDistance / 2;
            absMaxVelocity = absMaxAcceleration * timeToMaxVelocity;
        } else {
            // We reach max velocity, so we have a trapezoidal profile
            double distanceAtMaxVelocity = absTotalDistance - 2 * distanceToMaxVelocity;
            double timeAtMaxVelocity = distanceAtMaxVelocity / absMaxVelocity;
            totalTime = timeToMaxVelocity * 2 + timeAtMaxVelocity;
        }
        // Create the profile
        int arraySize = (int) Math.ceil(totalTime / 0.01) + 1;
        Profile[] profile = new Profile[arraySize];
        double currentTime = 0.0;
        int i = 0;
        while (i < arraySize - 1) {
            double currentPosition;
            double currentVelocity;
            double currentAcceleration;
            if (currentTime < timeToMaxVelocity) {
                // Acceleration phase
                currentAcceleration = direction * absMaxAcceleration;
                currentVelocity = direction * absMaxAcceleration * currentTime;
                currentPosition = startPosition + direction * 0.5 * absMaxAcceleration * currentTime * currentTime;
            } else if (currentTime < totalTime - timeToMaxVelocity) {
                // Constant velocity phase
                currentAcceleration = 0;
                currentVelocity = direction * absMaxVelocity;
                currentPosition = startPosition + direction * (distanceToMaxVelocity + absMaxVelocity * (currentTime - timeToMaxVelocity));
            } else {
                // Deceleration phase
                currentAcceleration = -direction * absMaxAcceleration;
                currentVelocity = direction * (absMaxVelocity - absMaxAcceleration * (currentTime - (totalTime - timeToMaxVelocity)));
                currentPosition = endPosition - direction * 0.5 * absMaxAcceleration * (totalTime - currentTime) * (totalTime - currentTime);
            }
            profile[i] = new Profile(currentPosition, currentVelocity, currentAcceleration, currentTime);
            currentTime += 0.01;
            i++;
        }
        // Add the final profile point to ensure it ends at (endPosition, 0, 0)
        profile[i] = new Profile(endPosition, 0, 0, totalTime);
        return profile;
    }
}
