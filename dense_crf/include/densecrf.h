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

#pragma once
#include "unary.h"
#include "labelcompatibility.h"
#include "objective.h"
#include "pairwise.h"
#include <vector>

/*  superpixel stuff */
#include "superpixel.h"

/**** DenseCRF ****/


class DenseCRF{
protected:
	// Number of variables
	int N_;
	// Number of labels
	int M_;
	
	// Store the unary term, my case is -log P(x)
	UnaryEnergy * unary_;
	UnaryEnergy * unary2_;
	
	
	// Store all pairwise potentials
	std::vector<PairwisePotential*> pairwise_;
	
	// Don't copy this object, bad stuff will happen
	DenseCRF( DenseCRF & o ){}
public:
	// Create a dense CRF model of size N with M labels
	DenseCRF( int N, int M );
	virtual ~DenseCRF();
	
	// Add  a pairwise potential defined over some feature space
	// The potential will have the form:    w*exp(-0.5*|f_i - f_j|^2)
	// The kernel shape should be captured by transforming the
	// features before passing them into this function
	// (ownership of LabelCompatibility will be transfered to this class)
	void addPairwiseEnergy( const MatrixXf & features, LabelCompatibility * function, KernelType kernel_type=DIAG_KERNEL, NormalizationType normalization_type=NORMALIZE_SYMMETRIC );
	
	// Add your own favorite pairwise potential (ownership will be transfered to this class)
	void addPairwiseEnergy( PairwisePotential* potential );
	
	// Set the unary potential (ownership will be transfered to this class)
	void setUnaryEnergy( UnaryEnergy * unary ); // size numclass*numallnodes.
	// Add a constant unary term each column is label distribution.
	void setUnaryEnergy( const MatrixXf & unary );
	void setUnaryEnergy2( const MatrixXf & unary );
	// Add a logistic unary term
	void setUnaryEnergy( const MatrixXf & L, const MatrixXf & f );
	
	// Run inference and return the probabilities
	MatrixXf inference( int n_iterations );
	MatrixXf inference_hierarchical( int n_iterations );
	MatrixXf inference_fuse ( int n_iterations );
	
	// Run MAP inference and return the map label for each pixel
	VectorXi map( int n_iterations );

	// Run MAP inference and return the map prob for each pixel
	VectorXi map_prob( int n_iterations ) const;	
	
	// Step by step inference
	MatrixXf startInference() const;
	void stepInference( MatrixXf & Q, MatrixXf & tmp1, MatrixXf & tmp2 ) const;
	VectorXi currentMap( const MatrixXf & Q ) const;
	
	// fuse Q distribution
	bool fuse_last_Q_dist=false;
	MatrixXf Q_last;
	bool Q_dist_initialized=false;
	float last_Q_weight=0.2;	
	
	
	// Learning functions
	// Compute the gradient of the objective function over mean-field marginals with
	// respect to the model parameters
	double gradient( int n_iterations, const ObjectiveFunction & objective, VectorXf * unary_grad, VectorXf * lbl_cmp_grad, VectorXf * kernel_grad=NULL ) const;
public: /* Debugging functions */
	// Compute the unary energy of an assignment l
	VectorXf unaryEnergy( const VectorXi & l );
	
	// Compute the pairwise energy of an assignment l (half of each pairwise potential is added to each of it's endpoints)
	VectorXf pairwiseEnergy( const VectorXi & l, int term=-1 );
	
	// Compute the KL-divergence of a set of marginals
	double klDivergence( const MatrixXf & Q ) const;

public: /* Parameters */
	VectorXf unaryParameters() const;
	void setUnaryParameters( const VectorXf & v );
	VectorXf labelCompatibilityParameters() const;
	void setLabelCompatibilityParameters( const VectorXf & v );
	VectorXf kernelParameters() const;
	void setKernelParameters( const VectorXf & v );
	
public: 
	/* high order related stuff */  	
	void set_ho(bool add_ho);
	bool addHO=false;
	bool decoupled_high_order=false;
	bool hierarchical_high_order=false; // inference both superpixel and grid together.
	MatrixXf high_order_sp_to_pix;  //for each node and each label, clique's potential on it. used in high order CRF
	MatrixXf high_order_pix_to_sp;  // pixel's high order potential on clique
	std::vector<SuperPixel*> all_3d_superpixels_;//NOTE not in densecrf class. need to first read then construct CRF.
	
	float ho_param1, ho_param2; 
	void calculate_higherorder_pot(const MatrixXf& Q, MatrixXf& temp_energy);
	void calculate_hierarchical_pot(const MatrixXf& Q_low,MatrixXf& Q_high,MatrixXf& temp_low_energy,MatrixXf& temp_high_energy);
	void calculate_hierar_highlevel_pot(const MatrixXf& Q_low,const MatrixXf& Q_high,MatrixXf& temp_high_energy);
	void calculate_hierar_lowlevel_pot(const MatrixXf& Q_low,MatrixXf& temp_low_energy);
	void readSegmentIndex(char *, int); // rea one image segmentation method
	void readStatsPotential(char *file_name, int num_of_layers);
	void mem_init_higherorder(); 
	void set_mem_init_higherorder(); 
	void del_mem_higherorder(); 

};

class DenseCRF3D:public DenseCRF{
// protected:
	
public:
	// Create a 3d dense CRF model of size N with M labels
	DenseCRF3D(int N,  int M);
	virtual ~DenseCRF3D();
	// Add a Gaussian pairwise potential with standard deviation sx and sy
	void addPairwiseGaussian( float sx, float sy, float sz, const MatrixXf& posList, LabelCompatibility* function=NULL, KernelType kernel_type=DIAG_KERNEL, \
							NormalizationType normalization_type=NORMALIZE_SYMMETRIC );
	
	// Add a Bilateral pairwise potential with spacial standard deviations sx, sy and color standard deviations sr,sg,sb
	void addPairwiseBilateral ( float sx, float sy, float sz, float sr, float sg, float sb, const MatrixXf& posList, const MatrixXf& colorList, 
				    LabelCompatibility* function=NULL, KernelType kernel_type=DIAG_KERNEL, NormalizationType normalization_type=NORMALIZE_SYMMETRIC );

	// Set the unary potential for a specific variable
	using DenseCRF::setUnaryEnergy;
};

class DenseCRF2D:public DenseCRF{
protected:
	// Width, height of the 2d grid
	int W_, H_;
	MatrixXf feature_Bilateral;  // some part could be precompute  by me
public:
	// Create a 2d dense CRF model of size W x H with M labels
	DenseCRF2D( int W, int H, int M );
	virtual ~DenseCRF2D();
	// Add a Gaussian pairwise potential with standard deviation sx and sy
	void addPairwiseGaussian( float sx, float sy, LabelCompatibility * function=NULL, KernelType kernel_type=DIAG_KERNEL, NormalizationType normalization_type=NORMALIZE_SYMMETRIC );
	
	void addBilateral_pos(float sx, float sy);
	bool init_pose_pairwise=false;
	// Add a Bilateral pairwise potential with spacial standard deviations sx, sy and color standard deviations sr,sg,sb
	void addPairwiseBilateral( float sx, float sy, float sr, float sg, float sb, const unsigned char * im, LabelCompatibility * function=NULL, 
							KernelType kernel_type=DIAG_KERNEL, NormalizationType normalization_type=NORMALIZE_SYMMETRIC );
	void addPairwiseBilateral( float sx, float sy, float sr, float sg, float sb, VectorX8u im, LabelCompatibility * function=NULL, 
							KernelType kernel_type=DIAG_KERNEL, NormalizationType normalization_type=NORMALIZE_SYMMETRIC );

	// Set the unary potential for a specific variable
	using DenseCRF::setUnaryEnergy;
};
