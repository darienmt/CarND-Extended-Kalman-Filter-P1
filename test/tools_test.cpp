#include "catch.hpp"
#include "../src/tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

SCENARIO( "on RMSE calculation" ) {

    GIVEN( "some estimations and ground truth" ) {

        Tools tools;
        vector<VectorXd> estimations;
        vector<VectorXd> ground_truth;

        WHEN( "estimations are empty" ) {
          VectorXd a_ground_truth(4);
          a_ground_truth << 1, 2, 3, 4;
          ground_truth.push_back( a_ground_truth );

          VectorXd rmse = tools.CalculateRMSE( estimations, ground_truth);

          REQUIRE( rmse.size() == 4 );
          REQUIRE( rmse.sum() == 0 );
        }

        WHEN( "ground-truths are empty" ) {
          VectorXd an_estimation(4);
          an_estimation << 1, 2, 3, 4;
          estimations.push_back( an_estimation );

          VectorXd rmse = tools.CalculateRMSE( estimations, ground_truth);

          REQUIRE( rmse.size() == 4 );
          REQUIRE( rmse.sum() == 0 );
        }

        WHEN( "estimations and ground-truth doesn't have the same size") {
          VectorXd an_estimation(4);
          an_estimation << 1, 2, 3, 4;
          estimations.push_back( an_estimation );
          estimations.push_back( an_estimation );
          VectorXd a_ground_truth(4);
          a_ground_truth << 1, 2, 3, 4;
          ground_truth.push_back( a_ground_truth );

          VectorXd rmse = tools.CalculateRMSE( estimations, ground_truth);

          REQUIRE( rmse.size() == 4 );
          REQUIRE( rmse.sum() == 0 );
        }

        WHEN( "estimation and ground-truth are good") {
          VectorXd an_estimation(4);
          an_estimation << 1, 2, 3, 4;
          estimations.push_back( an_estimation );
          estimations.push_back( an_estimation );
          VectorXd a_ground_truth(4);
          a_ground_truth << 4, 3, 2, 1;
          ground_truth.push_back( a_ground_truth );
          ground_truth.push_back( a_ground_truth );

          VectorXd rmse = tools.CalculateRMSE( estimations, ground_truth);

          REQUIRE( rmse.size() == 4 );
          REQUIRE( rmse(0) == 3 );
          REQUIRE( rmse(1) == 1 );
          REQUIRE( rmse(2) == 1 );
          REQUIRE( rmse(3) == 3 );
        }
    }
}

SCENARIO( "on Jacobian calculation" ) {
  GIVEN( "an state vector" ) {

    Tools tools;

    WHEN( "the state vector size is not 4 an error is reported" ) {
      VectorXd state(2);
      state << 1, 2;
      MatrixXd Hj = tools.CalculateJacobian(state);
    }

    WHEN( "px and py are zero an error should be printed.") {
      VectorXd state(4);
      state << 0, 0, 3, 4;
      MatrixXd Hj = tools.CalculateJacobian(state);
    }

    WHEN( "with other values the calculation should success.") {
      VectorXd state(4);
      state << 1, 2, 3, 4;
      MatrixXd Hj = tools.CalculateJacobian(state);
      REQUIRE( round(Hj(1, 1) * 10) == 2 );
      REQUIRE( round(Hj(1, 0) * 10) == -4 );
    }
  }
}
