#include <fly_px4/control_px4.hpp>

#include <functional>
#include <iostream>
#include <stdexcept>

using Target = mavros_msgs::PositionTarget;

namespace {
void require(bool condition, const char* description) {
    if (!condition) throw std::runtime_error(description);
}

bool near(double actual, double expected, double tolerance = 1e-9) {
    return std::fabs(actual - expected) <= tolerance;
}

// Capture the actual message emitted by ControlPx4, including the MAVROS
// boundary's coordinate scale and type mask, rather than a duplicate builder.
class OutputCapture {
public:
    explicit OutputCapture(ros::NodeHandle& nh) {
        subscriber_ = nh.subscribe<Target>("setpoint_raw/local", 10,
            [this](const Target::ConstPtr& message) {
                latest_ = *message;
                ++received_;
            });
        publisher_ = nh.advertise<Target>("setpoint_raw/local", 10);
        const auto deadline = ros::WallTime::now() + ros::WallDuration(5.0);
        while (publisher_.getNumSubscribers() == 0 && ros::ok()) {
            require(ros::WallTime::now() < deadline, "subscriber connection timed out");
            ros::spinOnce();
            ros::WallDuration(0.005).sleep();
        }
    }

    ros::Publisher publisher() const { return publisher_; }

    Target emit(const std::function<void()>& command) {
        const unsigned previous = received_;
        command();
        const auto deadline = ros::WallTime::now() + ros::WallDuration(2.0);
        while (received_ == previous && ros::ok()) {
            require(ros::WallTime::now() < deadline, "setpoint publication timed out");
            ros::spinOnce();
            ros::WallDuration(0.001).sleep();
        }
        require(received_ == previous + 1, "unexpected duplicate setpoint");
        return latest_;
    }

private:
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    Target latest_;
    unsigned received_ = 0;
};

void check_locked_z(const Target& target) {
    require(!(target.type_mask & Target::IGNORE_PZ), "height position must stay active");
    require((target.type_mask & Target::IGNORE_VZ) || near(target.velocity.z, 0.0),
            "locked height cannot have vertical velocity feedforward");
    require(target.type_mask & Target::IGNORE_AFZ,
            "locked height cannot have vertical acceleration feedforward");
    require(std::isfinite(target.position.z), "height reference must be finite");
}
}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_altitude_regression");
    ros::NodeHandle nh;
    try {
        OutputCapture output(nh);
        ControlPx4 control(output.publisher(), 2.0);
        auto target = output.emit([&] {
            control.fly_posvel_yaw(4.0, 6.0, 4.5, 0.0, 0.0, 0.0, 0.0);
        });
        require(near(target.position.z, 4.5), "initial cruise reference must be exact");
        target = output.emit([&] {
            control.fly_pva_xy_posez_yaw(14.0, -6.0, 4.5, 4.0, -2.0, 2.0, -4.0, 450.0);
        });
        require(target.type_mask == (Target::IGNORE_VZ | Target::IGNORE_AFZ |
                                     Target::IGNORE_YAW_RATE), "2D PVA mask changed other axes");
        require(target.coordinate_frame == Target::FRAME_LOCAL_NED,
                "MAVROS local coordinate frame changed");
        require(!target.header.stamp.isZero(), "PVA timestamp must be populated");
        require(near(target.position.x, 7.0) && near(target.position.y, -3.0) &&
                near(target.velocity.x, 2.0) && near(target.velocity.y, -1.0) &&
                near(target.acceleration_or_force.x, 1.0) &&
                near(target.acceleration_or_force.y, -2.0),
                "2D PVA must preserve horizontal derivatives and boundary scale");
        require(near(target.yaw, M_PI / 2.0, 1e-6), "PVA yaw normalization changed");
        check_locked_z(target);

        target = output.emit([&] {
            control.fly_posvel_yaw(4.0, 6.0, 4.5, 0.0, 0.0, 0.0, 0.0);
        });
        require(near(target.position.x, 2.0) && near(target.position.y, 3.0),
                "2D PVA must not change the existing XY position filter history");
        target = output.emit([&] {
            control.fly_velxy_posez_yaw(6.0, -4.0, 4.5, 0.0, 0.0);
        });
        require(near(target.velocity.x, 3.0) && near(target.velocity.y, -2.0),
                "PID velocity filtering or boundary scale changed");
        target = output.emit([&] {
            control.fly_pva_xy_posez_yaw(28.0, 36.0, 4.5, 26.0, -20.0,
                                       12.0, -10.0, -450.0, 0.8);
        });
        require(near(target.position.x, 14.0) && near(target.position.y, 18.0) &&
                near(target.velocity.x, 13.0) && near(target.velocity.y, -10.0) &&
                near(target.acceleration_or_force.x, 6.0) &&
                near(target.acceleration_or_force.y, -5.0),
                "horizontal PVA must bypass all existing horizontal filters");
        require(near(target.yaw, -M_PI / 2.0, 1e-6) && near(target.yaw_rate, 0.8, 1e-6) &&
                !(target.type_mask & Target::IGNORE_YAW_RATE),
                "tracking yaw and yaw-rate feedforward changed");
        target = output.emit([&] {
            control.fly_velxy_posez_yaw(6.0, -4.0, 4.5, 0.0, 0.0);
        });
        require(near(target.velocity.x, 3.0) && near(target.velocity.y, -2.0),
                "2D PVA must not change the existing XY velocity filter history");

        // 2026-10-14 返航冲击: 起飞进场时位置滤波停在首航点, 之后一直走 PVA,
        // 跟踪结束原地对准机头再调 fly_posvel_yaw 时首拍不能从旧首航点滑过来。
        {
            ControlPx4 handoff(output.publisher(), 2.0);
            output.emit([&] { handoff.fly_posvel_yaw(0.0, 0.0, 4.5, 0.0, 0.0, 0.0, 0.0); });
            for (int i = 0; i < 50; ++i)
                output.emit([&] { handoff.fly_posvel_yaw(60.0, 0.0, 4.5, 0.0, 0.0, 0.0, 0.0); });
            output.emit([&] {
                handoff.fly_pva_xy_posez_yaw(180.0, 30.0, 4.5, 0.0, 0.0, 0.0, 0.0, 0.0);
            });
            target = output.emit([&] {
                handoff.fly_posvel_yaw(180.0, 30.0, 4.5, 0.0, 0.0, 0.0, 0.0);
            });
            require(near(target.position.x, 90.0) && near(target.position.y, 15.0),
                    "position hold after PVA must start at its own target, not the stale waypoint");
            target = output.emit([&] {
                handoff.fly_posvel_yaw(182.0, 30.0, 4.5, 0.0, 0.0, 0.0, 0.0);
            });
            require(target.position.x > 90.0 && target.position.x < 91.0,
                    "position filter must still smooth consecutive position commands");

            output.emit([&] { handoff.fly_velxy_posez_yaw(6.0, 0.0, 4.5, 0.0, 0.0); });
            output.emit([&] {
                handoff.fly_pva_xy_posez_yaw(180.0, 30.0, 4.5, 0.0, 0.0, 0.0, 0.0, 0.0);
            });
            target = output.emit([&] { handoff.fly_velxyz(0.0, 0.0, 0.0); });
            require(near(target.velocity.x, 0.0) && near(target.velocity.y, 0.0),
                    "zero-velocity hold after PVA must not replay a stale PID velocity");
        }

        // Compare alternating flight modes with an uninterrupted Z-position
        // reference. A private filter in any mode causes an immediate mismatch.
        ControlPx4 reference(output.publisher());
        output.emit([&] {
            reference.fly_pva_xy_posez_yaw(0.0, 0.0, 4.5, 0.0, 0.0, 0.0, 0.0, 0.0);
        });
        double previous_z = 4.5;
        unsigned samples = 0;
        for (double goal : {2.7, 4.5, 2.7}) {
            const bool descending = goal < previous_z;
            for (unsigned i = 0; i < 150; ++i) {
                const auto expected = output.emit([&] {
                    reference.fly_pva_xy_posez_yaw(0.0, 0.0, goal,
                                                 0.0, 0.0, 0.0, 0.0, 0.0);
                });
                target = output.emit([&] {
                    switch (i % 4) {
                    case 0: control.fly_pva_xy_posez_yaw(14.0, -6.0, goal,
                                         4.0, -2.0, 2.0, -4.0, 90.0); break;
                    case 1: control.fly_velxy_posez_yaw(6.0, -4.0, goal, 90.0, 0.0); break;
                    case 2: control.fly_posvel_yaw(4.0, 6.0, goal,
                                         0.0, 0.0, 0.0, 90.0); break;
                    case 3: control.fly_hover(goal); break;
                    }
                });
                check_locked_z(target);
                require(near(target.position.z, expected.position.z),
                        "PVA/PID/position/hover handoff restored stale height history");
                require(std::fabs(target.position.z - previous_z) < 0.25,
                        "height reference jumped across a controller handoff");
                require(descending ? target.position.z <= previous_z + 1e-9
                                   : target.position.z >= previous_z - 1e-9,
                        "controller handoff reversed the height transition");
                require(descending ? target.position.z >= goal - 1e-9
                                   : target.position.z <= goal + 1e-9,
                        "height reference overshot the configured target");
                previous_z = target.position.z;
                ++samples;
            }
            require(near(previous_z, goal, 1e-6), "height reference failed to settle at YAML target");
        }

        target = output.emit([&] {
            control.fly_pva_yaw(9.0, -13.0, 1.1, 4.0, -2.0, 0.7,
                                2.0, -4.0, -0.4, -450.0, 2.0, 0.3);
        });
        require(target.type_mask == 0 && near(target.position.z, 2.0) &&
                near(target.velocity.z, 0.7) && near(target.acceleration_or_force.z, -0.4),
                "legacy RACER 3D PVA/min-height semantics changed");
        require(near(target.position.x, 4.5) && near(target.position.y, -6.5) &&
                near(target.velocity.x, 2.0) && near(target.acceleration_or_force.y, -2.0) &&
                near(target.yaw, -M_PI / 2.0, 1e-6) && near(target.yaw_rate, 0.3, 1e-6),
                "legacy RACER horizontal/yaw semantics changed");

        // The main controller's optional continuous reference must win over
        // every flight path's older altitude argument, without changing XY/yaw.
        ControlPx4 unified(output.publisher(), 2.0);
        ControlPx4 ordinary(output.publisher(), 2.0);
        const unsigned vertical_mask = Target::IGNORE_PZ | Target::IGNORE_VZ | Target::IGNORE_AFZ;
        for (unsigned i = 0; i < 6; ++i) {
            const auto command = [i](ControlPx4& controller) {
                switch (i) {
                case 0: controller.fly_pva_xy_posez_yaw(14.0, -6.0, 4.5,
                                            4.0, -2.0, 2.0, -4.0, 450.0, 0.8); break;
                case 1: controller.fly_velxy_posez_yaw(6.0, -4.0, 2.7, 90.0, 0.3); break;
                case 2: controller.fly_posvel_yaw(4.0, 6.0, 4.5,
                                            4.0, -2.0, 5.0, 90.0); break;
                case 3: controller.fly_hover(2.7); break;
                case 4: controller.fly_velxyz(6.0, -4.0, 5.0); break;
                case 5: controller.fly_pva_yaw(9.0, -13.0, 1.1, 4.0, -2.0, 5.0,
                                            2.0, -4.0, 8.0, -450.0, 2.0, 0.3); break;
                }
            };
            const auto expected = output.emit([&] { command(ordinary); });
            const double z = 3.25 + i * 0.01;
            unified.set_altitude_reference(z, 0.2, -0.3);
            target = output.emit([&] { command(unified); });
            require(near(target.position.z, z) && near(target.velocity.z, 0.2) &&
                    near(target.acceleration_or_force.z, -0.3) &&
                    !(target.type_mask & vertical_mask),
                    "a control mode bypassed the unified Z PVA reference");
            require((target.type_mask & ~vertical_mask) == (expected.type_mask & ~vertical_mask) &&
                    target.coordinate_frame == expected.coordinate_frame &&
                    near(target.position.x, expected.position.x) &&
                    near(target.position.y, expected.position.y) &&
                    near(target.velocity.x, expected.velocity.x) &&
                    near(target.velocity.y, expected.velocity.y) &&
                    near(target.acceleration_or_force.x, expected.acceleration_or_force.x) &&
                    near(target.acceleration_or_force.y, expected.acceleration_or_force.y) &&
                    near(target.yaw, expected.yaw) && near(target.yaw_rate, expected.yaw_rate),
                    "unified altitude changed a horizontal or yaw command");
        }
        unified.set_altitude_reference(2.7, 0.0, 0.0);
        target = output.emit([&] { unified.fly_hover(4.5); });
        require(near(target.position.z, 2.7) && near(target.velocity.z, 0.0) &&
                near(target.acceleration_or_force.z, 0.0) && !(target.type_mask & vertical_mask),
                "steady unified height retained conflicting vertical feedforward");
        unified.clear_altitude_reference();
        target = output.emit([&] { unified.fly_velxyz(0.0, 0.0, 0.0); });
        require((target.type_mask & (Target::IGNORE_PZ | Target::IGNORE_AFZ)) ==
                    (Target::IGNORE_PZ | Target::IGNORE_AFZ) &&
                !(target.type_mask & Target::IGNORE_VZ),
                "clearing altitude override must restore ground velocity-only mode");
        target = output.emit([&] {
            unified.fly_pva_yaw(9.0, -13.0, 1.1, 4.0, -2.0, 0.7,
                                2.0, -4.0, -0.4, -450.0, 2.0);
        });
        require(target.type_mask == Target::IGNORE_YAW_RATE && near(target.position.z, 2.0) &&
                near(target.velocity.z, 0.7) && near(target.acceleration_or_force.z, -0.4),
                "clearing altitude override must restore legacy 3D PVA");
        target = output.emit([&] { unified.fly_hover(6.0); });
        require(near(target.position.z, 6.0),
                "override must skip rather than contaminate the legacy Z filter history");
        std::cout << "PASS: " << samples << " actual MAVROS controller handoff samples; "
                  << "Z convergence, XY/yaw/scale preservation, legacy 3D PVA, "
                  << "six-mode unified altitude and override clearing" << std::endl;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << std::endl;
        return 1;
    }
    return 0;
}
