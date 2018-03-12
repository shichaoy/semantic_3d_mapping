/*
    Copyright (c) 2013, Philipp Krähenbühl
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the Stanford University nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Philipp Krähenbühl ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Philipp Krähenbühl BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "densecrf.h"
#include "permutohedral.h"
#include "util.h"
#include "pairwise.h"
#include <cmath>
#include <cstring>
#include <iostream>

#include <ctime>
#include <algorithm>    // std::min
#include <boost/concept_check.hpp>

template<typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
	return ((x.array() == x.array())).all();
}

/////////////////////////////
/////  Alloc / Dealloc  /////
/////////////////////////////
DenseCRF::DenseCRF(int N, int M) : N_(N), M_(M), unary_(0), unary2_(0) {
}
DenseCRF::~DenseCRF() {
	if (unary_)
		delete unary_;
	if (unary2_)
		delete unary2_;	
	for( unsigned int i=0; i<pairwise_.size(); i++ )
		delete pairwise_[i];
}
DenseCRF2D::DenseCRF2D(int W, int H, int M) : DenseCRF(W*H,M), W_(W), H_(H) {
}
DenseCRF2D::~DenseCRF2D() {
}

DenseCRF3D::DenseCRF3D(int N,  int M) : DenseCRF(N, M) {
}
DenseCRF3D::~DenseCRF3D() {
}
/////////////////////////////////
/////  Pairwise Potentials  /////
/////////////////////////////////
void DenseCRF::addPairwiseEnergy (const MatrixXf & features, LabelCompatibility * function, KernelType kernel_type, NormalizationType normalization_type) {
// 	assert( features.cols() == N_ ); //HACK  comment by me, for hierarchical use
	addPairwiseEnergy( new PairwisePotential( features, function, kernel_type, normalization_type ) );
}
void DenseCRF::addPairwiseEnergy ( PairwisePotential* potential ){
	pairwise_.push_back( potential );
}

void DenseCRF2D::addPairwiseGaussian ( float sx, float sy, LabelCompatibility * function, KernelType kernel_type, NormalizationType normalization_type ) {
	MatrixXf feature( 2, N_ );
	for( int j=0; j<H_; j++ )
		for( int i=0; i<W_; i++ ){
			feature(0,j*W_+i) = i / sx;
			feature(1,j*W_+i) = j / sy;
		}
	addPairwiseEnergy( feature, function, kernel_type, normalization_type );
}

void DenseCRF2D::addPairwiseBilateral ( float sx, float sy, float sr, float sg, float sb, const unsigned char* im, LabelCompatibility * function, KernelType kernel_type, NormalizationType normalization_type ) {
// 	MatrixXf feature_Bilateral( 5, N_ ); // some part of feature_Bilateral is fixed all the time
	if (!init_pose_pairwise)
	    addBilateral_pos(sx,sy);  
	for( int j=0; j<H_; j++ )
		for( int i=0; i<W_; i++ ){
// 			feature_Bilateral(0,j*W_+i) = i / sx;
// 			feature_Bilateral(1,j*W_+i) = j / sy;
			feature_Bilateral(2,j*W_+i) = im[(i+j*W_)*3+0] / sr;
			feature_Bilateral(3,j*W_+i) = im[(i+j*W_)*3+1] / sg;
			feature_Bilateral(4,j*W_+i) = im[(i+j*W_)*3+2] / sb;
		}
	addPairwiseEnergy( feature_Bilateral, function, kernel_type, normalization_type );
}


void DenseCRF2D::addPairwiseBilateral ( float sx, float sy, float sr, float sg, float sb, VectorX8u im, LabelCompatibility * function, KernelType kernel_type, NormalizationType normalization_type ) {
// 	MatrixXf feature( 5, N_ );
	if (!init_pose_pairwise)
	    addBilateral_pos(sx,sy);
	for( int j=0; j<H_; j++ )
		for( int i=0; i<W_; i++ ){
// 			feature_Bilateral(0,j*W_+i) = i / sx;
// 			feature_Bilateral(1,j*W_+i) = j / sy;
			feature_Bilateral(2,j*W_+i) = im[(i+j*W_)*3+0] / sr;
			feature_Bilateral(3,j*W_+i) = im[(i+j*W_)*3+1] / sg;
			feature_Bilateral(4,j*W_+i) = im[(i+j*W_)*3+2] / sb;
		}
	addPairwiseEnergy( feature_Bilateral, function, kernel_type, normalization_type );
}

void DenseCRF2D::addBilateral_pos(float sx,float sy)  // this is fixed, doesn't change with color
{
	feature_Bilateral.resize( 5, N_ );
	for( int j=0; j<H_; j++ )
	      for( int i=0; i<W_; i++ ){
		      feature_Bilateral(0,j*W_+i) = i / sx;
		      feature_Bilateral(1,j*W_+i) = j / sy;
	      }
	init_pose_pairwise=true;
}

void DenseCRF3D::addPairwiseGaussian( float sx, float sy, float sz, const MatrixXf& posList, 
					        LabelCompatibility * function, KernelType kernel_type, NormalizationType normalization_type ) {
// 	assert(posList.cols()==N_); // comment this for hierarchical crf
	MatrixXf feature( 3, posList.cols() );
	for (int i=0; i<posList.cols(); i++) {
		float x = posList(0,i);
		float y = posList(1,i);
		float z = posList(2,i);
		
		feature(0, i) = x / sx;
		feature(1, i) = y / sy;
		feature(2, i) = z / sz;
	}
	addPairwiseEnergy( feature, function, kernel_type, normalization_type );
}


void DenseCRF3D::addPairwiseBilateral ( float sx, float sy, float sz, float sr, float sg, float sb, const MatrixXf& posList, 
					  const MatrixXf& colorList, LabelCompatibility * function, KernelType kernel_type, NormalizationType normalization_type ) {
//       assert(posList.cols()==N_);  // comment this for hierarchical crf
//       assert(colorList.cols()==N_);
      assert(colorList.cols()==posList.cols());
      MatrixXf feature_Bilateral(6, posList.cols() );  // some part could be precompute  by me
      for (int i=0; i<posList.cols(); i++) {
		float x = posList(0,i);
		float y = posList(1,i);
		float z = posList(2,i);
		
		feature_Bilateral(0, i) = x / sx;
		feature_Bilateral(1, i) = y / sy;
		feature_Bilateral(2, i) = z / sz;
		
		feature_Bilateral(3, i) = colorList(0,i) / sr;
		feature_Bilateral(4, i) = colorList(1,i) / sg;
		feature_Bilateral(5, i) = colorList(2,i) / sb;
	}	
	addPairwiseEnergy( feature_Bilateral, function, kernel_type, normalization_type );
}


//////////////////////////////
/////  Unary Potentials  /////
//////////////////////////////
void DenseCRF::setUnaryEnergy ( UnaryEnergy * unary ) {
	if( unary_ ) delete unary_;
	unary_ = unary;
}
void DenseCRF::setUnaryEnergy( const MatrixXf & unary ) {
	setUnaryEnergy( new ConstUnaryEnergy( unary ) );
}
void DenseCRF::setUnaryEnergy2( const MatrixXf & unary ) {
	if( unary2_ ) delete unary2_;
	unary2_ = new ConstUnaryEnergy( unary );
}

void  DenseCRF::setUnaryEnergy( const MatrixXf & L, const MatrixXf & f ) {
	setUnaryEnergy( new LogisticUnaryEnergy( L, f ) );
}
///////////////////////
/////  Inference  /////
///////////////////////
void expAndNormalize ( MatrixXf & out, const MatrixXf & in ) {
	out.resize( in.rows(), in.cols() );
	for( int i=0; i<out.cols(); i++ ){
		VectorXf b = in.col(i);
		b.array() -= b.maxCoeff();
		b = b.array().exp();
		out.col(i) = b / b.array().sum();
	}
}

void sumAndNormalize( MatrixXf & out, const MatrixXf & in, const MatrixXf & Q ) {
	out.resize( in.rows(), in.cols() );
	for( int i=0; i<in.cols(); i++ ){
		VectorXf b = in.col(i);
		VectorXf q = Q.col(i);
		out.col(i) = b.array().sum()*q - b;
	}
}

MatrixXf DenseCRF::inference ( int n_iterations ) {
  	if (fuse_last_Q_dist)
	    return inference_fuse( n_iterations );
	
	if (addHO && hierarchical_high_order)
	    return inference_hierarchical( n_iterations );
	
	MatrixXf Q( M_, N_ ), tmp1, unary( M_, N_ ), tmp2;
	unary.fill(0);
	if( unary_ )
		unary = unary_->get();

	expAndNormalize( Q, -unary );
	
	for( int it=0; it<n_iterations; it++ ) 
	{
		tmp1 = -unary; //  tmp1, negative number  the higher, the larger prob
		if(addHO)
		{
			set_mem_init_higherorder();
			calculate_higherorder_pot(Q,tmp1);
			if (!decoupled_high_order)
			    tmp1 += high_order_sp_to_pix;  // minus high order potential energy
		}
		
		for( unsigned int k=0; k<pairwise_.size(); k++ ) {  // usually two. color and position
			pairwise_[k]->apply( tmp2, Q );  // each kind of features generate tmp2 (similary as unary, represent mutual energy)
			tmp1 -= tmp2;  // minus pairwise potential energy (minus a negative number)
		}
		expAndNormalize( Q, tmp1 ); //update Q (change energy into probability, each column independenty), doesn't change tmp1
	}
	return Q;
}


MatrixXf DenseCRF::inference_hierarchical( int n_iterations )
{
	int sp_num=all_3d_superpixels_.size();
	
	MatrixXf Q_low( M_, N_ ), tmp1_low, unary_low( M_, N_ ), tmp2_low;
	MatrixXf Q_high( M_, sp_num ), tmp1_high, unary_high( M_, sp_num), tmp2_high;

	std::cout<<"Hierarchical CRF low level size  "<<N_<<"  high level size  "<<sp_num<<std::endl;
	
	unary_low.fill(0);
	if( unary_ )
		unary_low = -unary_->get(); //NOTE, I keep a minus here, because all later operations use minus
	
	unary_high.fill(0); // high order unary set to 0, corresponding to clique's low cost is 0
	if( unary2_ )
		unary_high = -unary2_->get();

	std::clock_t start;

	expAndNormalize( Q_low,  unary_low);
	expAndNormalize( Q_high, unary_high);

	bool separate_two_level=true;
	if (separate_two_level)
	{
	      for( int it=0; it<n_iterations; it++ ) 
	      {
		      tmp1_low = unary_low;  // tmp1_low the higher, the larger prob
		      tmp1_high = unary_high;
// 		      set_mem_init_higherorder();  // if want to use  high_order_pix_to_sp etc.
		      
		      // first optimize high level nodes's label
		      calculate_hierar_highlevel_pot(Q_low,Q_high,tmp1_high);
// 		      tmp1_high -= high_order_pix_to_sp;
		      for( unsigned int k=2; k<4; k++ ) {  // for high level pixels
			      pairwise_[k]->apply( tmp2_high, Q_high );  // each kind of features generate tmp2 (similary as unary, represent mutual energy)
			      tmp1_high -= tmp2_high;  // minus pairwise potential energy
		      }
		      expAndNormalize( Q_high, tmp1_high ); //update Q (change energy into probability, each column independenty), doesn't change tmp1
		      for (int sp_id=0;sp_id<all_3d_superpixels_.size();sp_id++)
			    all_3d_superpixels_[sp_id]->max_label_prob=Q_high.col(sp_id).maxCoeff(&all_3d_superpixels_[sp_id]->label);
		      
		      // then fix high level nodes, only optimize low level nodes.
		      calculate_hierar_lowlevel_pot(Q_low,tmp1_low);
// 		      tmp1_low -= high_order_sp_to_pix;
		      for( unsigned int k=0; k<2; k++ ) {  // for low level pixels
			      pairwise_[k]->apply( tmp2_low, Q_low );  // each kind of features generate tmp2 (similary as unary, represent mutual energy)
			      tmp1_low -= tmp2_low;  // minus pairwise potential energy
		      }
		      expAndNormalize( Q_low, tmp1_low ); //update Q (change energy into probability, each column independenty), doesn't change tmp1
	      }
	}
	else
	{
	      for( int it=0; it<n_iterations; it++ ) 
	      {
		      tmp1_low = unary_low;
		      tmp1_high = unary_high;
// 		      set_mem_init_higherorder();

		      calculate_hierarchical_pot(Q_low,Q_high,tmp1_low,tmp1_high);
// 		      tmp1_low -= high_order_sp_to_pix;
// 		      tmp1_high -= high_order_pix_to_sp;

		      for( unsigned int k=0; k<2; k++ ) {  // for low level pixels
			      pairwise_[k]->apply( tmp2_low, Q_low );  // each kind of features generate tmp2 (similary as unary, represent mutual energy)
			      tmp1_low -= tmp2_low;  // minus pairwise potential energy
		      }

		      for( unsigned int k=2; k<4; k++ ) {  // for high level pixels
			      pairwise_[k]->apply( tmp2_high, Q_high );  // each kind of features generate tmp2 (similary as unary, represent mutual energy)
			      tmp1_high -= tmp2_high;  // minus pairwise potential energy
		      }
		      expAndNormalize( Q_low, tmp1_low ); //update Q (change energy into probability, each column independenty), doesn't change tmp1
		      expAndNormalize( Q_high, tmp1_high ); //update Q (change energy into probability, each column independenty), doesn't change tmp1
	      }
	}
	return Q_low;
}



void fuse_Q_dist(MatrixXf& curr_Q, const MatrixXf& last_Q, const float last_weight)
{
	curr_Q=last_weight*last_Q+(1-last_weight)*curr_Q;
	// normalize to sum to 1 in each column
	for( int i=0; i<curr_Q.cols(); i++ ){
		VectorXf b = curr_Q.col(i);
		curr_Q.col(i) = b / b.array().sum();
	}
}


MatrixXf DenseCRF::inference_fuse ( int n_iterations ) {
	MatrixXf Q, tmp1, unary( M_, N_ ), tmp2;
	unary.fill(0);
	if( unary_ )
	    unary = unary_->get();
		
	expAndNormalize( Q, -unary );
	if (Q_dist_initialized)  // fuse Q with last time
	    fuse_Q_dist(Q, Q_last, last_Q_weight);
	
	for( int it=0; it<n_iterations; it++ ) {
		tmp1 = -unary;  // right or not??? whether use Q
		for( unsigned int k=0; k<pairwise_.size(); k++ ) {
			pairwise_[k]->apply( tmp2, Q );
			tmp1 -= tmp2;
		}
		expAndNormalize( Q, tmp1 );
	}
	Q_last=Q;
	Q_dist_initialized=true;
	return Q;
}

VectorXi DenseCRF::map ( int n_iterations ) {
	// Run inference
	MatrixXf Q = inference( n_iterations );
	// Find the map
	return currentMap( Q );
}


// MatrixXf DenseCRF::map_prob( int n_iterations ) const{
//        return inference( n_iterations );
// }
      
///////////////////
/////  Debug  /////
///////////////////
VectorXf DenseCRF::unaryEnergy(const VectorXi & l) {
	assert( l.cols() == N_ );
	VectorXf r( N_ );
	r.fill(0.f);
	if( unary_ ) {
		MatrixXf unary = unary_->get();
		
		for( int i=0; i<N_; i++ )
			if ( 0 <= l[i] && l[i] < M_ )
				r[i] = unary( l[i], i );
	}
	return r;
}
VectorXf DenseCRF::pairwiseEnergy(const VectorXi & l, int term) {
	assert( l.cols() == N_ );
	VectorXf r( N_ );
	r.fill(0.f);
	
	if( term == -1 ) {
		for( unsigned int i=0; i<pairwise_.size(); i++ )
			r += pairwiseEnergy( l, i );
		return r;
	}
	
	MatrixXf Q( M_, N_ );
	// Build the current belief [binary assignment]
	for( int i=0; i<N_; i++ )
		for( int j=0; j<M_; j++ )
			Q(j,i) = (l[i] == j);
	pairwise_[ term ]->apply( Q, Q );
	for( int i=0; i<N_; i++ )
		if ( 0 <= l[i] && l[i] < M_ )
			r[i] =-0.5*Q(l[i],i );
		else
			r[i] = 0;
	return r;
}
MatrixXf DenseCRF::startInference() const{
	MatrixXf Q( M_, N_ );
	Q.fill(0);
	
	// Initialize using the unary energies
	if( unary_ )
		expAndNormalize( Q, -unary_->get() );
	return Q;
}
void DenseCRF::stepInference( MatrixXf & Q, MatrixXf & tmp1, MatrixXf & tmp2 ) const{
	tmp1.resize( Q.rows(), Q.cols() );
	tmp1.fill(0);
	if( unary_ )
		tmp1 -= unary_->get();
	
	// Add up all pairwise potentials
	for( unsigned int k=0; k<pairwise_.size(); k++ ) {
		pairwise_[k]->apply( tmp2, Q );
		tmp1 -= tmp2;
	}
	
	// Exponentiate and normalize
	expAndNormalize( Q, tmp1 );
}
VectorXi DenseCRF::currentMap( const MatrixXf & Q ) const{
	VectorXi r(Q.cols());
	// Find the map
	for( int i=0; i<N_; i++ ){
		int m;
		Q.col(i).maxCoeff( &m );
		r[i] = m;
	}
	return r;
}

// Compute the KL-divergence of a set of marginals
double DenseCRF::klDivergence( const MatrixXf & Q ) const {
	double kl = 0;
	// Add the entropy term
	for( int i=0; i<Q.cols(); i++ )
		for( int l=0; l<Q.rows(); l++ )
			kl += Q(l,i)*log(std::max( Q(l,i), 1e-20f) );
	// Add the unary term
	if( unary_ ) {
		MatrixXf unary = unary_->get();
		for( int i=0; i<Q.cols(); i++ )
			for( int l=0; l<Q.rows(); l++ )
				kl += unary(l,i)*Q(l,i);
	}
	
	// Add all pairwise terms
	MatrixXf tmp;
	for( unsigned int k=0; k<pairwise_.size(); k++ ) {
		pairwise_[k]->apply( tmp, Q );
		kl += (Q.array()*tmp.array()).sum();
	}
	return kl;
}

// Gradient computations
double DenseCRF::gradient( int n_iterations, const ObjectiveFunction & objective, VectorXf * unary_grad, VectorXf * lbl_cmp_grad, VectorXf * kernel_grad) const {
	// Run inference
	std::vector< MatrixXf > Q(n_iterations+1);
	MatrixXf tmp1, unary( M_, N_ ), tmp2;
	unary.fill(0);
	if( unary_ )
		unary = unary_->get();
	expAndNormalize( Q[0], -unary );
	for( int it=0; it<n_iterations; it++ ) {
		tmp1 = -unary;
		for( unsigned int k=0; k<pairwise_.size(); k++ ) {
			pairwise_[k]->apply( tmp2, Q[it] );
			tmp1 -= tmp2;
		}
		expAndNormalize( Q[it+1], tmp1 );
	}
	
	// Compute the objective value
	MatrixXf b( M_, N_ );
	double r = objective.evaluate( b, Q[n_iterations] );
	sumAndNormalize( b, b, Q[n_iterations] );

	// Compute the gradient
	if(unary_grad && unary_)
		*unary_grad = unary_->gradient( b );
	if( lbl_cmp_grad )
		*lbl_cmp_grad = 0*labelCompatibilityParameters();
	if( kernel_grad )
		*kernel_grad = 0*kernelParameters();
	
	for( int it=n_iterations-1; it>=0; it-- ) {
		// Do the inverse message passing
		tmp1.fill(0);
		int ip = 0, ik = 0;
		// Add up all pairwise potentials
		for( unsigned int k=0; k<pairwise_.size(); k++ ) {
			// Compute the pairwise gradient expression
			if( lbl_cmp_grad ) {
				VectorXf pg = pairwise_[k]->gradient( b, Q[it] );
				lbl_cmp_grad->segment( ip, pg.rows() ) += pg;
				ip += pg.rows();
			}
			// Compute the kernel gradient expression
			if( kernel_grad ) {
				VectorXf pg = pairwise_[k]->kernelGradient( b, Q[it] );
				kernel_grad->segment( ik, pg.rows() ) += pg;
				ik += pg.rows();
			}
			// Compute the new b
			pairwise_[k]->applyTranspose( tmp2, b );
			tmp1 += tmp2;
		}
		sumAndNormalize( b, tmp1.array()*Q[it].array(), Q[it] );
		
		// Add the gradient
		if(unary_grad && unary_)
			*unary_grad += unary_->gradient( b );
	}
	return r;
}
VectorXf DenseCRF::unaryParameters() const {
	if( unary_ )
		return unary_->parameters();
	return VectorXf();
}
void DenseCRF::setUnaryParameters( const VectorXf & v ) {
	if( unary_ )
		unary_->setParameters( v );
}
VectorXf DenseCRF::labelCompatibilityParameters() const {
	std::vector< VectorXf > terms;
	for( unsigned int k=0; k<pairwise_.size(); k++ )
		terms.push_back( pairwise_[k]->parameters() );
	int np=0;
	for( unsigned int k=0; k<pairwise_.size(); k++ )
		np += terms[k].rows();
	VectorXf r( np );
	for( unsigned int k=0,i=0; k<pairwise_.size(); k++ ) {
		r.segment( i, terms[k].rows() ) = terms[k];
		i += terms[k].rows();
	}	
	return r;
}
void DenseCRF::setLabelCompatibilityParameters( const VectorXf & v ) {
	std::vector< int > n;
	for( unsigned int k=0; k<pairwise_.size(); k++ )
		n.push_back( pairwise_[k]->parameters().rows() );
	int np=0;
	for( unsigned int k=0; k<pairwise_.size(); k++ )
		np += n[k];
	
	for( unsigned int k=0,i=0; k<pairwise_.size(); k++ ) {
		pairwise_[k]->setParameters( v.segment( i, n[k] ) );
		i += n[k];
	}	
}
VectorXf DenseCRF::kernelParameters() const {
	std::vector< VectorXf > terms;
	for( unsigned int k=0; k<pairwise_.size(); k++ )
		terms.push_back( pairwise_[k]->kernelParameters() );
	int np=0;
	for( unsigned int k=0; k<pairwise_.size(); k++ )
		np += terms[k].rows();
	VectorXf r( np );
	for( unsigned int k=0,i=0; k<pairwise_.size(); k++ ) {
		r.segment( i, terms[k].rows() ) = terms[k];
		i += terms[k].rows();
	}	
	return r;
}
void DenseCRF::setKernelParameters( const VectorXf & v ) {
	std::vector< int > n;
	for( unsigned int k=0; k<pairwise_.size(); k++ )
		n.push_back( pairwise_[k]->kernelParameters().rows() );
	int np=0;
	for( unsigned int k=0; k<pairwise_.size(); k++ )
		np += n[k];
	
	for( unsigned int k=0,i=0; k<pairwise_.size(); k++ ) {
		pairwise_[k]->setKernelParameters( v.segment( i, n[k] ) );
		i += n[k];
	}	
}


// start adding higher order potential related stuffs
void DenseCRF::set_ho(bool add_ho)
{
	addHO=add_ho;
}


void DenseCRF::set_mem_init_higherorder()
{
	if(addHO)
	{
		if (high_order_sp_to_pix.rows()!=M_)
		    high_order_sp_to_pix=MatrixXf::Zero(M_, N_);
		else
		    high_order_sp_to_pix.setZero();

		if (hierarchical_high_order)
		{
		    if (high_order_pix_to_sp.rows()!=M_)
			high_order_pix_to_sp=MatrixXf::Zero(M_,all_3d_superpixels_.size());
		    else
			high_order_pix_to_sp.setZero();
		}
	}
}

void DenseCRF::calculate_higherorder_pot(const MatrixXf& Q,MatrixXf& temp_energy) 
{
	if(!decoupled_high_order)
	{
		int segment_count=all_3d_superpixels_.size();
		MatrixXf h_norm = MatrixXf::Ones(M_, segment_count);  // label distribution for each superpixels, directly multiply all pixels probability of that class

		int curr_pix_label = 0, curr_pix_index; // int x, y; 

		VectorXf::Index max_label;

		float higher_order_prob;
		int pixels_num_in_superpixel = 0; 
		for(int sp_id = 0; sp_id < segment_count; sp_id++)  // for each superpixel
		{
			pixels_num_in_superpixel = all_3d_superpixels_[sp_id]->pixel_indexes.size();
			for(int k = 0; k < M_; k++) // for all the class
			{
				higher_order_prob = 0.0; 
				for(int j = 0; j < pixels_num_in_superpixel; j++) // for each pixel in the superpixel //TODO change to vector production
				{
					curr_pix_index = all_3d_superpixels_[sp_id]->pixel_indexes[j];
					higher_order_prob = higher_order_prob + Q(k,curr_pix_index); // change product to sum, avoid underflow
				}
				h_norm(k,sp_id) = higher_order_prob; // multiply all pixel prob in one superpixel
			}
		}

		double alpha = 0.5;
		float cost_l = 1;  //10
		float cost_max = 5;  //100
		float high_order_weight = 1; // weight relative to unary and pairwise. TODO should be large?
		for(int sp_id = 0; sp_id < segment_count; sp_id++)   // for each superpixel
		{
			pixels_num_in_superpixel = all_3d_superpixels_[sp_id]->pixel_indexes.size();
			for(int j = 0; j < pixels_num_in_superpixel; j++)   //  for each pixel in the superpixel
			{
			    VectorXf::Index max_label;
			    curr_pix_index = all_3d_superpixels_[sp_id]->pixel_indexes[j]; 
			    for(int k = 0; k < M_; k++)  //  for all the class
			    {
				higher_order_prob = h_norm(k,sp_id)-Q(k,curr_pix_index)+0.0001;
				higher_order_prob = higher_order_prob/float(pixels_num_in_superpixel);
			      
				// higher_order_prob is very small, so this is very close to cost_max
				high_order_sp_to_pix(k,curr_pix_index) += higher_order_prob*cost_l+(1-higher_order_prob)*cost_max;

				// our proposed.   superpixel's label influce the clique,
// 				high_order_sp_to_pix(k,curr_pix_index) -= ( all_3d_superpixels_[sp_id]->label_distri).dot( Q.col(curr_pix_index) ) * 100; // 100 is a weght
// 				if (k==all_3d_superpixels_[sp_id]->label)
// 				    high_order_sp_to_pix(k,curr_pix_index) -= higher_order_prob*all_3d_superpixels_[sp_id]->label_distri[k]*100;
			    }
			}
		}
		high_order_sp_to_pix = high_order_sp_to_pix*high_order_weight;
	}
	else  // change robust Pn model into pairwise+auxiliary node. but assume auxiliary's node is fixed.
	{
		int curr_pix_label = 0, curr_pix_index; // int x, y;

		for(int sp_id = 0; sp_id < all_3d_superpixels_.size(); sp_id++)  // for each superpixel
		{
			for(int j = 0; j < all_3d_superpixels_[sp_id]->pixel_indexes.size(); j++) // for each pixel in the superpixel //TODO change to vector production
			{
				curr_pix_index = all_3d_superpixels_[sp_id]->pixel_indexes[j];
				float max_prob = Q.col(curr_pix_index).maxCoeff(&curr_pix_label);
				if (curr_pix_label==all_3d_superpixels_[sp_id]->label)
				    temp_energy(curr_pix_label,curr_pix_index) -= all_3d_superpixels_[sp_id]->max_label_prob*10.0; 
				  // only one label (superpixel's label) has some energy.
// 				    high_order_sp_to_pix(curr_pix_label,curr_pix_index) -= 10.0; //if label the same, give negative energy
			}
		}
	}
}

//both minimize_x_y  E(x,y)
void DenseCRF::calculate_hierarchical_pot(const MatrixXf& Q_low,MatrixXf& Q_high,MatrixXf& temp_low_energy,MatrixXf& temp_high_energy)
{
	int curr_pix_label = 0, curr_pix_index; // int x, y;
	int curr_sp_label = 0;

	for(int sp_id = 0; sp_id < all_3d_superpixels_.size(); sp_id++)  // for each superpixel
	{
		float max_prob_sp = Q_high.col(sp_id).maxCoeff(&curr_sp_label);
		for(int j = 0; j < all_3d_superpixels_[sp_id]->pixel_indexes.size(); j++) // for each pixel in the superpixel
		{
			curr_pix_index = all_3d_superpixels_[sp_id]->pixel_indexes[j];
			float max_prob_pix = Q_low.col(curr_pix_index).maxCoeff(&curr_pix_label);
			if (curr_pix_label==curr_sp_label)
			{
// 			      high_order_sp_to_pix(curr_pix_label,curr_pix_index) -= max_prob_sp*0.2; 
// 			      high_order_pix_to_sp(curr_sp_label,sp_id) -= max_prob_pix*0.2;
			      temp_low_energy(curr_pix_label,curr_pix_index) -= max_prob_sp*0.2;  // directly minus, faster
			      temp_high_energy(curr_sp_label,sp_id) -= max_prob_pix*0.2;
			}
		}
	}
}

//only minimize_y E(x,y)
void DenseCRF::calculate_hierar_highlevel_pot(const MatrixXf& Q_low,const MatrixXf& Q_high,MatrixXf& temp_high_energy)
{
	int curr_pix_label = 0, curr_pix_index; // int x, y;
	int curr_sp_label = 0;

	for(int sp_id = 0; sp_id < all_3d_superpixels_.size(); sp_id++)  // for each superpixel
	{
		float max_prob_sp = Q_high.col(sp_id).maxCoeff(&curr_sp_label);
		for(int j = 0; j < all_3d_superpixels_[sp_id]->pixel_indexes.size(); j++) // for each pixel in the superpixel
		{
			curr_pix_index = all_3d_superpixels_[sp_id]->pixel_indexes[j];
			float max_prob_pix = Q_low.col(curr_pix_index).maxCoeff(&curr_pix_label);
			if (curr_pix_label==curr_sp_label) // if the same label, encourage, give negative energy.  could also penalize for different label.
			{
// 			      high_order_pix_to_sp(curr_sp_label,sp_id) -= max_prob_pix*0.2;
			      temp_high_energy(curr_sp_label,sp_id) += 0.1;  // faster, directly add, originally, increase it to encourage it
			}
		}
	}
}

// using the minimal y, min_x E(x,y)
void DenseCRF::calculate_hierar_lowlevel_pot(const MatrixXf& Q_low,MatrixXf& temp_low_energy)
{
	int curr_pix_label = 0, curr_pix_index; // int x, y;

	std::clock_t begin = std::clock();
	for(int sp_id = 0; sp_id < all_3d_superpixels_.size(); sp_id++)  // for each superpixel
	{
		for(int j = 0; j < all_3d_superpixels_[sp_id]->pixel_indexes.size(); j++) // for each pixel in the superpixel
		{
			curr_pix_index = all_3d_superpixels_[sp_id]->pixel_indexes[j];
			float max_prob_pix = Q_low.col(curr_pix_index).maxCoeff(&curr_pix_label);
			if (curr_pix_label==all_3d_superpixels_[sp_id]->label)
			{
			      temp_low_energy(curr_pix_label,curr_pix_index) += 5.0;
			}
		}
	}
}
