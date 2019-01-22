#include commander.hpp


namespace handIn
{
	

Eigen::VectorXd* commander::getRefsForCircularFlight(const double zFlight, const double omega, const double turnRadius)
{
	if(!isGreen_)
	{return 0;}
	if(omega==0) //rotational speed of circle
	{

	}


}


Eigen::VectorXd* commander::leastSquares(const Eigen::MatrixXd &A, const Eigen::VectorXd &z)
{
	
}


} //end ns

