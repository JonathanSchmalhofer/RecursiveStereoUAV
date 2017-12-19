// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_TRAJECTORY3D_ROS_TO_AIRSIM_H_
#define FLATBUFFERS_GENERATED_TRAJECTORY3D_ROS_TO_AIRSIM_H_

#include "flatbuffers/flatbuffers.h"

#include "Point_generated.h"
#include "Pose_generated.h"
#include "Quaternion_generated.h"
#include "Trajectory3DPointStamped_generated.h"
#include "time_ros_to_airsim_generated.h"

namespace ros_to_airsim {

struct Trajectory3D;

struct Trajectory3D FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  enum {
    VT_TRAJECTORY = 4
  };
  const flatbuffers::Vector<flatbuffers::Offset<Trajectory3DPointStamped>> *trajectory() const {
    return GetPointer<const flatbuffers::Vector<flatbuffers::Offset<Trajectory3DPointStamped>> *>(VT_TRAJECTORY);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_TRAJECTORY) &&
           verifier.Verify(trajectory()) &&
           verifier.VerifyVectorOfTables(trajectory()) &&
           verifier.EndTable();
  }
};

struct Trajectory3DBuilder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_trajectory(flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Trajectory3DPointStamped>>> trajectory) {
    fbb_.AddOffset(Trajectory3D::VT_TRAJECTORY, trajectory);
  }
  explicit Trajectory3DBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  Trajectory3DBuilder &operator=(const Trajectory3DBuilder &);
  flatbuffers::Offset<Trajectory3D> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<Trajectory3D>(end);
    return o;
  }
};

inline flatbuffers::Offset<Trajectory3D> CreateTrajectory3D(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<Trajectory3DPointStamped>>> trajectory = 0) {
  Trajectory3DBuilder builder_(_fbb);
  builder_.add_trajectory(trajectory);
  return builder_.Finish();
}

inline flatbuffers::Offset<Trajectory3D> CreateTrajectory3DDirect(
    flatbuffers::FlatBufferBuilder &_fbb,
    const std::vector<flatbuffers::Offset<Trajectory3DPointStamped>> *trajectory = nullptr) {
  return ros_to_airsim::CreateTrajectory3D(
      _fbb,
      trajectory ? _fbb.CreateVector<flatbuffers::Offset<Trajectory3DPointStamped>>(*trajectory) : 0);
}

inline const ros_to_airsim::Trajectory3D *GetTrajectory3D(const void *buf) {
  return flatbuffers::GetRoot<ros_to_airsim::Trajectory3D>(buf);
}

inline bool VerifyTrajectory3DBuffer(
    flatbuffers::Verifier &verifier) {
  return verifier.VerifyBuffer<ros_to_airsim::Trajectory3D>(nullptr);
}

inline void FinishTrajectory3DBuffer(
    flatbuffers::FlatBufferBuilder &fbb,
    flatbuffers::Offset<ros_to_airsim::Trajectory3D> root) {
  fbb.Finish(root);
}

}  // namespace ros_to_airsim

#endif  // FLATBUFFERS_GENERATED_TRAJECTORY3D_ROS_TO_AIRSIM_H_