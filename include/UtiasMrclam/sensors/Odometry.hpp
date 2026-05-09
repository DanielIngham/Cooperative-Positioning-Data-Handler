namespace utias::mrclam::sensor {
class Odometry {
public:
  Odometry() = delete;
  Odometry(Odometry &&) = default;
  Odometry(const Odometry &) = default;
  Odometry &operator=(Odometry &&) = default;
  Odometry &operator=(const Odometry &) = default;
  ~Odometry() = default;

  Odometry(double forward_velocity, double angular_velocity)
      : forward_velocity_{forward_velocity},
        angular_velocity_{angular_velocity} {};

private:
  double forward_velocity_;
  double angular_velocity_;
};
} // namespace utias::mrclam::sensor
