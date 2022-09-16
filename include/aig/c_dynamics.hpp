/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a biped robot.
 */

#ifndef CENTROIDAL_DYNAMICS
#define CENTROIDAL_DYNAMICS

// clang-format off
#include <Eigen/Dense>
#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/spatial/se3.hpp"
#include <map>
// clang-format on

namespace dyno {

  enum contact{surface, point};

  struct ContactSettings {
      public:

        contact type;
        double mu, gu;
        double half_length, half_width;
        std::string frame_name;

      friend std::ostream &operator<<(std::ostream &out,
                                      const ContactSettings &obj) {
        std::string type;
        if (obj.type == contact::surface) type = "surface";
        else if (obj.type == contact::point) type = "point";

        out << "Contact "<<type<<":\n";
        out << "    mu: " << obj.mu << "\n";
        out << "    gu: " << obj.gu << "\n";
        if (obj.type == contact::point) out<<std::endl;
        else if (obj.type == contact::surface) 
        {
          out << "    Surface half_length: " << obj.half_length << "\n";
          out << "    Surface half_width: " << obj.half_width << std::endl;
        }
        return out;
      }

      friend bool operator==(const ContactSettings &lhs,
                              const ContactSettings &rhs) {
        bool test = true;
        test &= lhs.type == rhs.type;
        test &= lhs.frame_name == rhs.frame_name;
        test &= lhs.mu == rhs.mu;
        test &= lhs.gu == rhs.gu;

        if (lhs.type == contact::point) return test;

        test &= lhs.half_length == rhs.half_length;
        test &= lhs.half_width == rhs.half_width;
        return test;
      }
  };

    struct DynoSettings {
      public:
      /**
       * @brief This must contain either a valid path to the urdf file or the
       * content of this file in a string.
       */
      std::string urdf = "";

      friend std::ostream &operator<<(std::ostream &out,
                                      const DynoSettings &obj) {
        out << "DynoSettings:\n";
        out << "    urdf: " << obj.urdf << std::endl;
        return out;
      }

      friend bool operator==(const DynoSettings &lhs,
                              const DynoSettings &rhs) {
        bool test = true;
        test &= lhs.urdf == rhs.urdf;
        return test;
      }
  };

  class Dyno{

    private:
      DynoSettings settings_;
      Eigen::Matrix2d S_;
      pinocchio::Model model_;
      pinocchio::Data data_;
      std::map<std::string, ContactSettings> contacts_;
      double mass_;

    //Internal variables:
      Eigen::Vector3d groundCoMForce_, groundCoMTorque_, nonCoPTorque_, weight_;
      Eigen::Vector3d acom_;
      Eigen::Vector2d cop_;
      Eigen::Vector3d dL_;
      Eigen::Vector3d L_;
      Eigen::Vector2d n_;

    public:
      
      Dyno();
      Dyno(const DynoSettings settings);
      void initialize(const DynoSettings settings);

      void computeDynamics(const Eigen::VectorXd &posture,
                            const Eigen::VectorXd &velocity,
                            const Eigen::VectorXd &acceleration,
                            const Eigen::Matrix<double, 6, 1> &externalWrench =
                            Eigen::Matrix<double, 6, 1>::Zero(),
                            bool flatHorizontalGround = true);

      void computeNL(const double &w, const Eigen::VectorXd &posture,
                      const Eigen::VectorXd &velocity,
                      const Eigen::VectorXd &acceleration,
                      const Eigen::Matrix<double, 6, 1> &externalWrench =
                            Eigen::Matrix<double, 6, 1>::Zero(),
                      bool flatHorizontalGround = true);

      void computeNL(const double &w);

      // GETTERS
      /// @brief Please call computeDynamics first.
      const Eigen::Vector3d &getAMVariation() { return dL_; }
      const Eigen::Vector3d &getCoM() { return data_.com[0]; }
      const Eigen::Vector3d &getVCoM() { return data_.vcom[0]; }
      const Eigen::Vector3d &getACoM() { return acom_; }
      const Eigen::Vector3d &getAM() { return L_; }
      const Eigen::Vector2d &getCoP() { return cop_; }
      const Eigen::Vector2d &getNL() { return n_; }
      const Eigen::Vector3d &getGroundCoMForce() { return groundCoMForce_; }
      const Eigen::Vector3d &getGroundCoMTorque() { return groundCoMTorque_; }
  };


}


#endif // CENTROIDAL_DYNAMICS