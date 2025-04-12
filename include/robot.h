#ifndef INCLUDE_INCLUDE_ROBOT_H_
#define INCLUDE_INCLUDE_ROBOT_H_

class Robot {
public:
	Robot();
	Robot(Robot &&) = default;
	Robot(const Robot &) = default;
	Robot &operator=(Robot &&) = default;
	Robot &operator=(const Robot &) = default;
	~Robot();

private:
	
};

Robot::Robot() {
}

Robot::~Robot() {
}

#endif  // INCLUDE_INCLUDE_ROBOT_H_
