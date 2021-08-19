#include "graphOptimize.h"

namespace slam{

namespace optimizer{

GraphOptimize::GraphOptimize()
{

}

GraphOptimize::~GraphOptimize()
{

}

void GraphOptimize::createOptimizer()
{
	auto linearSolver = g2o::make_unique<SlamLinearSolver>();

	linearSolver->setBlockOrdering(false);

	g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( g2o::make_unique<SlamBlockSolver>( std::move(linearSolver) ) );

	optimizer.setAlgorithm(solver);
}

void GraphOptimize::addVertex( const Eigen::Vector3f &pose, const int id )
{
	VertexSE2 *vertex = new VertexSE2();
	
	vertex->setId( id );

	vertex->setEstimate( SE2( pose(0), pose(1), pose(2) ) );

	optimizer.addVertex( vertex );

	std::cout<<"add a vertex to the optimizer ..."<<std::endl;
}

void GraphOptimize::addEdge( const Eigen::Vector3f &delta, 
			     const int from, 
                             const int to,  
                             Eigen::Matrix3d &information )
{
	EdgeSE2 *edge = new EdgeSE2();
	
	edge->vertices()[0] = optimizer.vertex( from );
	edge->vertices()[1] = optimizer.vertex( to );
	
	SE2 measurement( delta[0], delta[1], delta[2] );
	edge->setMeasurement( measurement );

	edge->setId( edgeCount );
	
	edge->setInformation( information );
	
	optimizer.addEdge( edge );

	edgeCount ++;
	
	std::cout<<"add a edge to the optimize ... "<<std::endl;
}

int GraphOptimize::execuateGraphOptimization()
{
	optimizer.initializeOptimization();
	
	int iteration = 0;
	
	iteration = optimizer.optimize(100);

	std::cout<<"execuate the graph optimization ... "<<std::endl;
	
	return iteration; 
}

void GraphOptimize::getOptimizedResults()
{
	g2o::SparseOptimizer::VertexContainer nodes = optimizer.activeVertices();

	// TODO ....
}

}

}

