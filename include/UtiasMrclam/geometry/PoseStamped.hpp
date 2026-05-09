#include "UtiasMrclam/geometry/Pose.hpp"

namespace utias::mrclam::geo {

class PoseStamped {
public:
  PoseStamped() = delete;
  PoseStamped(PoseStamped &&) = default;
  PoseStamped(const PoseStamped &) = default;
  PoseStamped &operator=(PoseStamped &&) = default;
  PoseStamped &operator=(const PoseStamped &) = default;
  ~PoseStamped() = default;

  PoseStamped(double time, double x, double y, double orientation)
      : time_stamp{time}, pose_{x, y, orientation} {}

private:
  double time_stamp;
  Pose pose_;
};
} // namespace utias::mrclam::geo
