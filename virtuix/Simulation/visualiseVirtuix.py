import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from collections import deque
import argparse

# Emulate C# Vector3 type
class Vector3:
    def __init__(self, x, y, z):
        self.vector = np.array([x, y, z], dtype=float)

    def x(self):
        return self.vector[0]
    
    def y(self):
        return self.vector[1]
    
    def z(self):
        return self.vector[2]
    
    def __add__(self, other):
        if isinstance(other, Vector3):
            self.vector += other.vector
        else:
            self.vector += other
        return self
    
    def __sub__(self, other):
        if isinstance(other, Vector3):
            self.vector -= other.vector
        else:
            self.vector -= other
        return self
    
    def __mul__(self, other):
        self.vector *= other
        return self
    
    def __div__(self, other):
        if isinstance(other, Vector3):
            self.vector /= other.vector
        else:
            self.vector /= other
        return self
    
    def __truediv__(self, other):
        if isinstance(other, Vector3):
            self.vector /= other.vector
        else:
            self.vector /= other
        return self
    
    def __lt__(self, other):
        return self.vector < other
    
    def __str__(self):
        # return f"{self.x()},\t{self.y()},\t{self.z()}"
        return f"{self.x()},\t{self.z()}"
    
    def round(self):
        self.vector = np.round(self.vector)
        return self


# Emulate Unity script
class TeleopVirtuixCommunication:
    def __init__(self, movementThreshold, movementMultiplier, noStepThreshold, stepIncrement, speedLimit, movingAverageWindow, rotationThreshold):
        self.history = deque([])
        self.movementThreshold = movementThreshold
        self.movingAverageSum = Vector3(0, 0, 0)
        self.movementMultiplier = movementMultiplier
        self.noStepCount = 0
        self.noStepThreshold = noStepThreshold
        self.previousMovement = Vector3(0, 0, 0)
        self.movingAverageWindow = movingAverageWindow
        self.speedLimit = speedLimit # Vector3
        self.stepIncrement = stepIncrement
        self.rotationThreshold = rotationThreshold # Degrees instead of radians
        self.previousRotation = 0

    def __applyMovingAverage(self, movement):
        self.history.append(movement)
        if len(self.history) > self.movingAverageWindow:
            self.history.popleft()
        return np.mean(self.history)
    
    def __applySteppedMovement(self, movement):
        scaled = movement / self.stepIncrement
        rounded = scaled.round()
        stepped = rounded * self.stepIncrement
        return stepped
        
    def __applySpeedLimit(self, movement):
        magnitude = np.abs(movement.vector)
        sign = np.sign(movement.vector)
        limited = np.minimum(magnitude, self.speedLimit.vector) * sign
        return Vector3(limited[0], limited[1], limited[2])

    def __applyMovementThreshold(self, movement):
        if (abs(movement.z()) > self.movementThreshold):
            self.noStepCount = 0
            self.previousMovement = movement
        else:
            self.noStepCount += 1
            if self.noStepCount > self.noStepThreshold:
                movement = Vector3(0, 0, 0)
            else:
                movement = self.previousMovement
        return movement

    def update(self, forward, strafe, rotation):
        # Movment
        
        movement = forward + strafe
        movement = self.__applyMovingAverage(movement)
        movement *= self.movementMultiplier
        movement = self.__applySteppedMovement(movement)
        movement = self.__applySpeedLimit(movement)
        movement = self.__applyMovementThreshold(movement)

        # Rotation
        if abs(rotation - self.previousRotation) > self.rotationThreshold:
            self.previousRotation = rotation
        else:
            rotation = self.previousRotation

        return movement, rotation


# Simulate filtering of recorded virtuix data
class Simulation:
    def __init__(self, virtuixObj):
        self.virtuixObj = virtuixObj

    def simulate(self, data):
        movement = []
        rotation = []
        for i in range(len(data)):
            # Get recorded data
            forward = Vector3(data['Forward.x'][i], data['Forward.y'][i], data['Forward.z'][i])       
            strafe = Vector3(data['Strafe.x'][i], data['Strafe.y'][i], data['Strafe.z'][i])
            angle = data['Angle'][i]
            _mov, _rot = self.virtuixObj.update(forward, strafe, angle)
            movement.append(_mov)
            rotation.append(_rot)
        return movement, rotation


def main():

    # Get data from file argument
    parser = argparse.ArgumentParser(description='CSV data file input.')
    parser.add_argument('-d', '--data', type=str, required=True, help='CSV file name (required)')
    args = parser.parse_args()
    csv_file = args.data
    df = pd.read_csv(csv_file)

    # Convert timestamp to datetime
    df['Timestamp'] = pd.to_datetime(df['Timestamp'], unit='ms')

    '''
    Simulation
    '''

    # Run
    virtuix = TeleopVirtuixCommunication(0, 10, 3, 0.2, Vector3(0.6, 0, 0.4), 10, 11.46)
    sim = Simulation(virtuix)
    simMovement, simRotation = sim.simulate(df)

    '''
    Plot Movement
    '''

    # Create figure with two subplots side by side
    _, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # Plot Forward.z
    ax1.plot(df['Timestamp'], df['Forward.z'], marker='o', linestyle='-', markersize=1)
    ax1.set_xlabel('Time')
    ax1.set_ylabel("Virtuix output x-component")
    ax1.set_title('Raw')
    ax1.set_xticklabels([])  # Removes x-axis numbers
    ax1.grid(True)

    # Plot simulated z
    simMovement = [vec.z() for vec in simMovement]
    ax2.plot(df['Timestamp'], simMovement, marker='x', linestyle='-', markersize=1, color='orange')
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Linear velocity (m/s)')
    ax2.set_title('Processed')
    ax2.set_xticklabels([])  # Removes x-axis numbers
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

    '''
    Plot Rotation
    '''

    # Create figure with two subplots side by side
    _, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    # Plot Forward.z
    ax1.plot(df['Angle'], marker='o', linestyle='-', markersize=1,)
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Virtuix output angle (deg)')
    ax1.set_title('Raw')
    ax1.set_xticklabels([])  # Removes x-axis numbers
    ax1.grid(True)

    # Plot simulated z
    ax2.plot(simRotation, marker='x', linestyle='-', markersize=1, color='orange')
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Angle (deg)')
    ax2.set_title('Processed')
    ax2.set_xticklabels([])  # Removes x-axis numbers
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()