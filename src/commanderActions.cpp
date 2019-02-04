#include "commander.hpp"
#include <cstdio>
#include <vector>

#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

namespace handIn
{

DEFINE_double(robust_threshold, 0.0, "Robust loss parameter. Set to 0 for "
              "normal squared error (no robustification).");
class DistanceFromCircleCost {
 public:
  DistanceFromCircleCost(double xx, double yy) : xx_(xx), yy_(yy) {}
  template <typename T> bool operator()(const T* const x,
                                        const T* const y,
                                        const T* const m,  // r = m^2
                                        T* residual) const {
    // Since the radius is parameterized as m^2, unpack m to get r.
    T r = *m * *m;

    // Get the position of the sample in the circle's coordinate system.
    T xp = xx_ - *x;
    T yp = yy_ - *y;

    // It is tempting to use the following cost:
    //
    //   residual[0] = r - sqrt(xp*xp + yp*yp);
    //
    // which is the distance of the sample from the circle. This works
    // reasonably well, but the sqrt() adds strong nonlinearities to the cost
    // function. Instead, a different cost is used, which while not strictly a
    // distance in the metric sense (it has units distance^2) it produces more
    // robust fits when there are outliers. This is because the cost surface is
    // more convex.
    residual[0] = r*r - xp*xp - yp*yp;
    return true;
  }

 private:
  // The measured x,y coordinate that should be on the circle.
  double xx_, yy_;
};


class DistanceFromInitCost {
 public:
  	DistanceFromInitCost(Eigen::MatrixXd xx, Eigen::MatrixXd yy) : xx_(xx), yy_(yy) {}
 	template <typename T> bool operator()(const T* const xV,
                                        const T* const yV,
                                        const T* const mV,  // r = m^2
                                        T* residual) const {
    	// Since the radius is parameterized as m^2, unpack m to get r.
    	T r = *mV * *mV;

    	// Get the position of the sample in the circle's coordinate system.
	    T xp = xx_ - *xV;
   		T yp = yy_ - *yV;
    	residual[0] = r*r - xp*xp - yp*yp;
    	return true;
  	}

 	private:
  	// The measured x,y coordinate that should be on the circle.
 	Eigen::MatrixXd xx_, yy_;
};


Eigen::VectorXd commander::getRefsForCircularFlight(const double zFlight, const double omega, const double turnRadius)
{
	if(!isGreen_)
	{return Eigen::Vector3d(9001,9001,9001);}
	
	Eigen::MatrixXd retvec;
	retvec.resize(numQuads_*2*3, 1); //pos, vel

	//note: the lastGesture_ -> currentGesture_ comparison should be done in switch callback
	if(!gestureHasBeenInitialized_)
	{
		Eigen::MatrixXd currentPoses;
		currentPoses.resize(numQuads_*3,1);
		Eigen::Vector3d tmp;
		for(int ij=0; ij<numQuads_; ij++)
		{
			quadPoseContainer_[ij]->getPosPointer(&tmp);
			currentPoses(3*ij+0) = tmp(0);
			currentPoses(3*ij+1) = tmp(1);
			currentPoses(3*ij+2) = tmp(2);
		}
		gestureHasBeenInitialized_=true;

		/*//Figure out which quad should occupy which position in the new "circle"
		Eigen::MatrixXd A;
		A.resize(numQuads_*3, numQuads_*3);
		for(int ij=0; ij<numQuads_*3; ij++) {A(ij,ij)=1;} //Eigen::Diag does not fully initialize off-diagonal entries which causes errors when inverting
		Eigen::VectorXd tmp2;
		tmp2.resize(numQuads_*3);
		tmp2 = leastSquares(A,currentPoses);*/

		int numconfigs = numQuads_;
		for(int ij=0; ij<numconfigs; ij++)
		{

		}

	}
	if(omega!=0) //rotational speed of circle
	{

	}
	return Eigen::Vector3d(9001,9001,9001);
}


/*
Eigen::VectorXd commander::getMinimumDistanceInitialConfiguration(const Eigen::VectorXd &gesture)
{}
*/


//Overloaded function handle to handle publishing references. Call as:
// 2 inputs:  P,V
// 3 inputs:  P,V,A
// 4 inputs:  P,V,A,y
void commander::publishAllReferences(const Eigen::MatrixXd &posRefs, const Eigen::MatrixXd &velRefs)
{
	mg_msgs::PVA pvaMsg;

	for(int ij=0; ij<numQuads_; ij++)
	{
		pvaMsg.Pos.x = posRefs(3*ij+0);
		pvaMsg.Pos.y = posRefs(3*ij+1);
		pvaMsg.Pos.z = posRefs(3*ij+2);
		pvaMsg.Vel.x = velRefs(3*ij+0);
		pvaMsg.Vel.y = velRefs(3*ij+1);
		pvaMsg.Vel.z = velRefs(3*ij+2);
		pvaMsg.Acc.x = 0;
		pvaMsg.Acc.y = 0;
		pvaMsg.Acc.z = 0;
		pvaMsg.yaw = 0.0;

		pvaPub_[ij].publish(pvaMsg);
	}
}


void commander::publishAllReferences(const Eigen::MatrixXd &posRefs, const Eigen::MatrixXd &velRefs, const Eigen::MatrixXd &accRefs)
{
	mg_msgs::PVA pvaMsg;

	for(int ij=0; ij<numQuads_; ij++)
	{
		pvaMsg.Pos.x = posRefs(3*ij+0);
		pvaMsg.Pos.y = posRefs(3*ij+1);
		pvaMsg.Pos.z = posRefs(3*ij+2);
		pvaMsg.Vel.x = velRefs(3*ij+0);
		pvaMsg.Vel.y = velRefs(3*ij+1);
		pvaMsg.Vel.z = velRefs(3*ij+2);
		pvaMsg.Acc.x = accRefs(3*ij+0);
		pvaMsg.Acc.y = accRefs(3*ij+1);
		pvaMsg.Acc.z = accRefs(3*ij+2);
		pvaMsg.yaw = 0.0;

		pvaPub_[ij].publish(pvaMsg);
	}
}


void commander::publishAllReferences(const Eigen::MatrixXd &posRefs, const Eigen::MatrixXd &velRefs, const Eigen::MatrixXd &accRefs,
	const Eigen::MatrixXd &yawRefs)
{
	mg_msgs::PVA pvaMsg;

	for(int ij=0; ij<numQuads_; ij++)
	{
		pvaMsg.Pos.x = posRefs(3*ij+0);
		pvaMsg.Pos.y = posRefs(3*ij+1);
		pvaMsg.Pos.z = posRefs(3*ij+2);
		pvaMsg.Vel.x = velRefs(3*ij+0);
		pvaMsg.Vel.y = velRefs(3*ij+1);
		pvaMsg.Vel.z = velRefs(3*ij+2);
		pvaMsg.Acc.x = accRefs(3*ij+0);
		pvaMsg.Acc.y = accRefs(3*ij+1);
		pvaMsg.Acc.z = accRefs(3*ij+2);
		pvaMsg.yaw = yawRefs(ij);

		pvaPub_[ij].publish(pvaMsg);
	}
}


Eigen::VectorXd commander::leastSquares(const Eigen::MatrixXd &A, const Eigen::VectorXd &z)
{
	//size checks here
	if(A.rows()-z.rows() !=0)
	{
		return (A.transpose()*A).inverse()*A.transpose()*z;
	}
	else
	{return Eigen::Vector3d(9001,9001,9001);} //standard error return
}



void commander::testCeres()
{
    double x, y, r;
    if (scanf("%lg %lg %lg", &x, &y, &r) != 3) {
        fprintf(stderr, "Couldn't read first line.\n");
    }
    fprintf(stderr, "Got x, y, r %lg, %lg, %lg\n", x, y, r);

    // Save initial values for comparison.
    double initial_x = x;
    double initial_y = y;
    double initial_r = r;

    // Parameterize r as m^2 so that it can't be negative.
    double m = sqrt(r);

    Problem problem;

    // Configure the loss function.
    LossFunction* loss = NULL;
    if (FLAGS_robust_threshold) {
        loss = new CauchyLoss(FLAGS_robust_threshold);
    }

    // Add the residuals.
    double xx, yy;
    int num_points = 0;
    while (scanf("%lf %lf\n", &xx, &yy) == 2) {
        CostFunction *cost =
                new AutoDiffCostFunction<DistanceFromCircleCost, 1, 1, 1, 1>(
                        new DistanceFromCircleCost(xx, yy));
        problem.AddResidualBlock(cost, loss, &x, &y, &m);
        num_points++;
    }

    std::cout << "Got " << num_points << " points.\n";

    // Build and solve the problem.
    Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::DENSE_QR;
    Solver::Summary summary;
        Solve(options, &problem, &summary);

    // Recover r from m.
    r = m * m;

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";
    std::cout << "y : " << initial_y << " -> " << y << "\n";
    std::cout << "r : " << initial_r << " -> " << r << "\n";
}


} //end ns

