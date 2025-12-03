#ifndef REXROV_FGO_NAV_BODY_VELOCITY_FACTOR_H
#define REXROV_FGO_NAV_BODY_VELOCITY_FACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>

namespace rexrov_fgo_nav {

/**
 * BodyVelocityFactor - 机体坐标系速度约束因子
 *
 * 与ESKF中DVL更新的原理相同：
 * - DVL测量的是机体坐标系速度 v_body
 * - 状态中的速度是世界坐标系速度 v_world
 * - 误差 = v_body_measured - R_wb^T * v_world
 *
 * 这样DVL观测不依赖于姿态估计的准确性，
 * 因为误差是在机体坐标系中计算的。
 */
class BodyVelocityFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
private:
    gtsam::Vector3 measured_body_velocity_;  // DVL测量的机体坐标系速度

public:
    using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>;

    BodyVelocityFactor() = default;

    /**
     * 构造函数
     * @param poseKey 姿态状态的key (X)
     * @param velKey 世界坐标系速度状态的key (V)
     * @param measured_body_velocity DVL测量的机体坐标系速度
     * @param model 噪声模型
     */
    BodyVelocityFactor(gtsam::Key poseKey, gtsam::Key velKey,
                       const gtsam::Vector3& measured_body_velocity,
                       const gtsam::SharedNoiseModel& model)
        : Base(model, poseKey, velKey),
          measured_body_velocity_(measured_body_velocity) {}

    ~BodyVelocityFactor() override = default;

    /**
     * 计算误差
     * error = measured_body_velocity - R_wb^T * v_world
     *       = measured_body_velocity - R_bw * v_world
     *
     * 其中 R_wb 是从body到world的旋转矩阵 (pose.rotation())
     *      R_bw = R_wb^T 是从world到body的旋转矩阵
     */
    gtsam::Vector evaluateError(
        const gtsam::Pose3& pose,
        const gtsam::Vector3& world_velocity,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const override {

        // 使用GTSAM内置的旋转方法，自动计算正确的雅可比
        gtsam::Matrix33 H_rot;   // unrotate对Rot3(第一个参数this)的雅可比
        gtsam::Matrix33 H_vel;   // unrotate对Point3(第二个参数p)的雅可比

        // unrotate: 将世界坐标系向量转到机体坐标系 v_body = R^T * v_world
        gtsam::Vector3 predicted_body_velocity = pose.rotation().unrotate(
            world_velocity,
            H1 ? &H_rot : nullptr,
            H2 ? &H_vel : nullptr);

        // 误差 (在机体坐标系中)
        gtsam::Vector3 error = measured_body_velocity_ - predicted_body_velocity;

        // 雅可比矩阵
        if (H1) {
            // H1 是 3x6 矩阵 (对Pose3的雅可比)
            // Pose3 = (Rot3, Point3)，GTSAM的顺序是 [rotation(3), translation(3)]
            // error = v_meas - unrotate(v_world)
            // ∂error/∂pose = -[∂unrotate/∂rot, 0]
            *H1 = gtsam::Matrix::Zero(3, 6);
            H1->block<3, 3>(0, 0) = -H_rot;  // 对rotation部分 (前3列)
            // translation部分为0 (后3列)，因为unrotate不依赖位置
        }

        if (H2) {
            // ∂error/∂v_world = -∂unrotate/∂v_world
            *H2 = -H_vel;
        }

        return error;
    }

    /// 克隆
    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new BodyVelocityFactor(*this)));
    }

    /// 打印
    void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "BodyVelocityFactor" << std::endl;
        std::cout << "  measured body velocity: " << measured_body_velocity_.transpose() << std::endl;
        Base::print("", keyFormatter);
    }
};

} // namespace rexrov_fgo_nav

#endif // REXROV_FGO_NAV_BODY_VELOCITY_FACTOR_H
