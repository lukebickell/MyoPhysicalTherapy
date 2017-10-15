// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>
#include <time.h>
#include <map>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>

//Constants
int FREQUENCY = 10;
int TOLERANCE = 2;
int MAX_STRIKES = 2;

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:
	uint64_t timestamp;
	DataCollector()
		: onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
	{
	}

	// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
	void onUnpair(myo::Myo* myo, uint64_t timestamp)
	{
		// We've lost a Myo.
		// Let's clean up some leftover state.
		roll_w = 0;
		pitch_w = 0;
		yaw_w = 0;
		onArm = false;
		isUnlocked = false;
	}

	// onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
	// as a unit quaternion.
	void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
	{
		using std::atan2;
		using std::asin;
		using std::sqrt;
		using std::max;
		using std::min;

		// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
		float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
			1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
		float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
		float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
			1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

		// Convert the floating point angles in radians to a scale from 0 to 18.
		roll_w = static_cast<int>((roll + (float)M_PI) / (M_PI * 2.0f) * 18);
		pitch_w = static_cast<int>((pitch + (float)M_PI / 2.0f) / M_PI * 18);
		yaw_w = static_cast<int>((yaw + (float)M_PI) / (M_PI * 2.0f) * 18);
		this->timestamp = timestamp;
	}

	// onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
	// making a fist, or not making a fist anymore.
	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
	{
		currentPose = pose;

		if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
			// Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
			// Myo becoming locked.
			myo->unlock(myo::Myo::unlockHold);

			// Notify the Myo that the pose has resulted in an action, in this case changing
			// the text on the screen. The Myo will vibrate.
			myo->notifyUserAction();
		}
		else {
			// Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
			// are being performed, but lock after inactivity.
			//myo->unlock(myo::Myo::unlockTimed);
		}
	}

	// onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
	// arm. This lets Myo know which arm it's on and which way it's facing.
	void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
		myo::WarmupState warmupState)
	{
		onArm = true;
		whichArm = arm;
	}

	// onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
	// it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
	// when Myo is moved around on the arm.
	void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
	{
		onArm = false;
	}

	// onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
	void onUnlock(myo::Myo* myo, uint64_t timestamp)
	{
		isUnlocked = true;
	}

	// onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
	void onLock(myo::Myo* myo, uint64_t timestamp)
	{
		isUnlocked = false;
	}

	// There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
	// For this example, the functions overridden above are sufficient.

	// We define this function to print the current values that were updated by the on...() functions above.
	void print()
	{
		// Clear the current line
		std::cout << '\r';

		// Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
		/*std::cout << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
		<< '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
		<< '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';*/
		std::cout << '[' << "roll: " << roll_w << ']'
			<< '[' << "pitch: " << pitch_w << ']'
			<< '[' << "yaw: " << yaw_w << ']';

		if (onArm) {
			// Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.

			// Pose::toString() provides the human-readable name of a pose. We can also output a Pose directly to an
			// output stream (e.g. std::cout << currentPose;). In this case we want to get the pose name's length so
			// that we can fill the rest of the field with spaces below, so we obtain it as a string using toString().
			std::string poseString = currentPose.toString();

			std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
				<< '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
				<< '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';
		}
		else {
			// Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
			std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
		}

		std::cout << std::flush;
	}

	// These values are set by onArmSync() and onArmUnsync() above.
	bool onArm;
	myo::Arm whichArm;

	// This is set by onUnlocked() and onLocked() above.
	bool isUnlocked;

	// These values are set by onOrientationData() and onPose() above.
	int roll_w, pitch_w, yaw_w;
	myo::Pose currentPose;
};
struct EulerAngle
{
	int roll = 0;
	int pitch = 0;
	int yaw = 0;

	bool equals(EulerAngle ua)
	{
		if (std::abs(roll - ua.roll) <=TOLERANCE 
			&& std::abs(pitch - ua.pitch)<=TOLERANCE
			&& std::abs(yaw - ua.yaw)<=TOLERANCE)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	std::string toJSONString()
	{
		return std::string("\n{\n\"roll\": ") + std::to_string(roll) +
			",\n\"pitch\": " + std::to_string(pitch) +
			",\n\"yaw\": " + std::to_string(yaw) + "\n}";

	}
};
class Gesture
{
public:
	std::vector<EulerAngle>* values;

	Gesture(std::vector<EulerAngle>* val)
	{
		values = val;
	}

	Gesture()
	{
		values = new std::vector<EulerAngle>();
	}
	
	bool equals(EulerAngle euler, int n)
	{
		if ((values->at(n)).equals(euler))
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	int getNumSteps()
	{
		return values->size();
	}

	std::string toJSONString()
	{
		std::string builder = "{\n\"gesture\": [";

		for (int i = 0; i < values->size(); i++)
		{
			builder += values->at(i).toJSONString() + ", ";
		}
		builder += "\b\b]\n}";
		return builder;
	}
};

class GestureRecorder
{
private:
	myo::Myo* myo;
	myo::Hub* hub;
	DataCollector* collector;
	Gesture * lastGesture;

public:
	GestureRecorder(myo::Myo* myo, myo::Hub* hub, DataCollector* collector)
	{
		this->myo = myo;
		this->hub = hub;
		this->collector = collector;
		hub->addListener(collector);
		lastGesture = new Gesture();
	}

	void reset()
	{
		lastGesture = new Gesture;
		lastGesture->values = new std::vector<EulerAngle>();
	}

	void record()
	{
		reset();
		EulerAngle lastAngle;
		bool minorChange = false;

		while (true)
		{
			EulerAngle newAngle = lastAngle;
			hub->run(1000/FREQUENCY);
			if (collector->currentPose == myo::Pose::doubleTap)
			{
				break;
			}
			/*
			if (collector->currentPose == myo::Pose::waveOut)
			{
				minorChange = false;
				std::cout << "Reset" << std::endl;
				reset();
			}
			*/
			if (lastAngle.pitch == collector->pitch_w && lastAngle.roll == collector->roll_w && lastAngle.yaw == collector->yaw_w)
			{
				minorChange = false;
				continue;
			}
			if (std::abs(lastAngle.pitch - collector->pitch_w) <= 1 && std::abs(lastAngle.roll - collector->roll_w) <= 1 && std::abs(lastAngle.yaw - collector->yaw_w) <= 1 && minorChange)
			{
				minorChange = true;
				continue;
			}
			minorChange = false;
			newAngle.pitch = collector->pitch_w;
			newAngle.roll = collector->roll_w;
			newAngle.yaw = collector->yaw_w;

			std::cout << "\r[R: " << newAngle.roll << "][P: " << newAngle.pitch << "][Y: " << newAngle.yaw << "]";

			lastGesture->values->push_back(newAngle);

			//std::cout << '\r' << collector->currentPose.toString();

			lastAngle = newAngle;
		}
	}

	void printLastGesture()
	{
		for (int i = 0; i < lastGesture->values->size(); i++)
		{
			EulerAngle angle = lastGesture->values->at(i);
			std::cout << "\nR: " << angle.roll << " P: " << angle.pitch << " Y: " << angle.yaw;
		}
	}

	Gesture * getGesture()
	{
		return lastGesture;
	}

};

class GestureListener
{
private:
	myo::Myo* myo;
	myo::Hub* hub;
	DataCollector* collector;
	Gesture * lastGesture;

public:
	GestureListener(myo::Myo* myo, myo::Hub* hub, DataCollector* collector)
	{
		this->myo = myo;
		this->hub = hub;
		this->collector = collector;
		hub->addListener(collector);
		lastGesture = new Gesture();
	}

	bool isGesture(Gesture * gesture)
	{
		EulerAngle lastAngle;
		int correct = 0;
		int numSteps = gesture->getNumSteps();
		int strikes = 0;
		bool minorChange = false;
		while (correct < numSteps)
		{
			if (collector->currentPose == myo::Pose::waveOut)
			{
				break;
			}
			EulerAngle newAngle = lastAngle;
			hub->run(1000/FREQUENCY);
			if (lastAngle.pitch == collector->pitch_w && lastAngle.roll == collector->roll_w && lastAngle.yaw == collector->yaw_w)
			{
				minorChange = false;
				continue;
			}
			if (std::abs(lastAngle.pitch - collector->pitch_w) <= 1 && std::abs(lastAngle.roll - collector->roll_w) <= 1 && std::abs(lastAngle.yaw - collector->yaw_w) <= 1 && minorChange)
			{
				minorChange = true;
				continue;
			}
			minorChange = false;
			newAngle.pitch = collector->pitch_w;
			newAngle.roll = collector->roll_w;
			newAngle.yaw = collector->yaw_w;

			std::cout << "\r[R: " << newAngle.roll << "][P: " << newAngle.pitch << "][Y: " << newAngle.yaw << "]";

			if (gesture->equals(newAngle, correct))
			{
				correct++;
			}
			else if (strikes>=MAX_STRIKES)
			{
				correct = 0;
				strikes = 0;
			}
			else
			{
				strikes++;
			}

			//std::cout << '\r' << collector->currentPose.toString();

			lastAngle = newAngle;
		}
		return true;
	}
	/*
	std::string isGesture(std::map<std::string>, Gesture * gesture>)
	{
		EulerAngle lastAngle;
		int correct = 0;
		int numSteps = gesture->getNumSteps();
		int strikes = 0;
		bool minorChange = false;
		while (correct < numSteps)
		{
			if (collector->currentPose == myo::Pose::waveOut)
			{
				break;
			}
			EulerAngle newAngle = lastAngle;
			hub->run(1000 / FREQUENCY);
			if (lastAngle.pitch == collector->pitch_w && lastAngle.roll == collector->roll_w && lastAngle.yaw == collector->yaw_w)
			{
				minorChange = false;
				continue;
			}
			if (std::abs(lastAngle.pitch - collector->pitch_w) <= 1 && std::abs(lastAngle.roll - collector->roll_w) <= 1 && std::abs(lastAngle.yaw - collector->yaw_w) <= 1 && minorChange)
			{
				minorChange = true;
				continue;
			}
			minorChange = false;
			newAngle.pitch = collector->pitch_w;
			newAngle.roll = collector->roll_w;
			newAngle.yaw = collector->yaw_w;

			std::cout << "\r[R: " << newAngle.roll << "][P: " << newAngle.pitch << "][Y: " << newAngle.yaw << "]";

			if (gesture->equals(newAngle, correct))
			{
				correct++;
			}
			else if (strikes >= MAX_STRIKES)
			{
				correct = 0;
				strikes = 0;
				std::cout << "RESET" << std::endl;
			}
			else
			{
				strikes++;
				std::cout << "strike" << std::endl;
			}

			//std::cout << '\r' << collector->currentPose.toString();

			lastAngle = newAngle;
		}
		return true;
	}
	*/
	void printLastGesture()
	{
		for (int i = 0; i < lastGesture->values->size(); i++)
		{
			EulerAngle angle = lastGesture->values->at(i);
			std::cout << "\nR: " << angle.roll << " P: " << angle.pitch << " Y: " << angle.yaw;
		}
	}

};

class Gestures
{
public:
	std::map<std::string, Gesture*> gest;
	std::string keyAt(int n)
	{
		int i = 0;
		for (std::map<std::string, Gesture*>::const_iterator it = gest.begin(); it != gest.end(); ++it)
		{
			if (i == n)
			{
				return it->first;
			}
		}
	}
	int getSize()
	{
		int i = 0;
		for (std::map<std::string, Gesture*>::const_iterator it = gest.begin(); it != gest.end(); ++it)
		{
			i++;
		}
		return i;
	}
};

int main(int argc, char** argv)
{
	// We catch any exceptions that might occur below -- see the catch statement for more details.
	try {

		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.
		myo::Hub *hub = new myo::Hub("com.example.hello-myo");

		std::cout << "Attempting to find a Myo..." << std::endl;

		// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
		// immediately.
		// waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
		// if that fails, the function will return a null pointer.
		myo::Myo* myo = hub->waitForMyo(10000);

		// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
		}

		// We've found a Myo.
		std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

		// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
		DataCollector * collector = new DataCollector();
		GestureRecorder * recorder = new GestureRecorder(myo, hub, collector);
		GestureListener * listener = new GestureListener(myo, hub, collector);
		Gestures gestures;

		while (true)
		{
			std::cout << "\n1. Therapist - Record a gesture \n2. Patient - Perform reps of a gesture" << std::endl;
			int inputNum;
			char saveChar;
			std::cin >> inputNum;

			// Record gesture
			if (inputNum == 1) {
				recorder->record();
				std::cout << "Do you want to save (Y/N)? ";
				std::cin >> saveChar;
				while (saveChar != 'Y' && saveChar != 'N' && saveChar != 'y' && saveChar != 'n')
				{
					std::cout << "Invalid!" << std::endl;
					std::cin >> saveChar;
				}
				if (saveChar == 'Y' || saveChar == 'y')
				{
					std::cout << "\nGesture recorded! Enter a name for the gesture: " << std::endl;
					std::string name;
					std::cin >> name;

					gestures.gest[name] = recorder->getGesture();
					std::cout << "\nGesture " << name << " saved!" << std::endl;
				}
				else
				{
					std::cout << "\nGesture discarded!" << std::endl;
				}
			}
			else if (inputNum == 2)
			{
				int input = 0;
				int totalReps = 0;
				int reps = 0;

				for (int i = 0; i < gestures.getSize(); i++)
				{
					std::cout << i+1 << ". " << gestures.keyAt(i) << std::endl;
				}
				std::cin >> input;
				while (input > gestures.getSize()+1)
				{
					std::cout << "Incorrect input!" << std::endl;
					std::cin >> input;
				}
				//Sorry for the sloppy code. It's 6:14am...
				std::cout << "How many reps would you like to perform? ";
				std::cin >> totalReps;
				while (reps <= totalReps)
				{
					std::cout << "Reps: " << reps << " / " << totalReps << std::endl;
					listener->isGesture(gestures.gest[gestures.keyAt(input - 1)]);
					reps++;
				}
			}
			else {
				std::cout << "Incorrect input!" << std::endl;
				continue;
			}
		}


		// If a standard exception occurred, we print out its message and exit.
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}
}