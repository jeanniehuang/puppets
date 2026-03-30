#pragma once

/** \file
 * Defines some utility functions needed by the PuppetWarp algorithm(s), but
 * which aren't a good fit anywhere else.
 */


#include "optimtools/all.h"

BEGIN_OPTIMTOOLS_NAMESPACE

/**
 * Collects a few utility functions needed by the Fast Automatic Skinning
 * Transformations algorithm, mostly related to computing the rotation
 * clusters.
 */
template<typename Scalar>
struct fast_arap_utils {
private:
	typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
	typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
	typedef Eigen::SparseMatrix<Scalar> SparseMatrix;
	typedef typename mesh_traits<Scalar,2>::TriangleVector TriangleVector;
	typedef typename mesh_traits<Scalar,2>::Triangle Triangle;

public:
	/**
	 * Greedily clusters points by their Euciadean distance.
	 */
	static void greedily_cluster_points(
		IntVector& out_point_clusters,
		const Matrix& P,
		Int num_clusters
	) {
		Int num_points = P.cols();
		Int num_dimensions = P.rows();
		Assert(num_clusters > 0) << "cluster_points must produce at least one cluster" << raise;
		Assert(num_clusters <= P.cols()) << "cluster_points cannot produce more clusters than the total number of points" << raise;

		Matrix cluster_centers = Matrix::Zero(num_dimensions, num_clusters);

		// choose the first cluster center to be the point furthest from 0
		Vector dists = P.colwise().squaredNorm();
		out_point_clusters.resize(num_points);
		{
			// pick the representative vertex for the cluster
			Int ind;
			dists.maxCoeff(&ind);
			auto center_point = P.col(ind);
			cluster_centers.col(0) = center_point;

			// update the distances
			for(Int point_ind = 0; point_ind < num_points; ++point_ind) {
				dists[point_ind] = (P.col(point_ind) - center_point).squaredNorm();
				out_point_clusters[point_ind] = 0;
			}
		}

		// greedily add the rest of the clusters
		for(Int cluster_ind = 1; cluster_ind < num_clusters; ++cluster_ind) {
			// pick the representative vertex for the cluster
			Int ind;
			dists.maxCoeff(&ind);
			auto center_point = P.col(ind);
			cluster_centers.col(cluster_ind) = center_point;

			// update the distances
			for(Int point_ind = 0; point_ind < num_points; ++point_ind) {
				Scalar d = (P.col(point_ind) - center_point).squaredNorm();
				if(d < dists[point_ind]) {
					dists[point_ind] = d;
					out_point_clusters[point_ind] = cluster_ind;
				}
			}
		}
	}


	/**
	 * This function takes as input a matrix implicitly described by 
	 * \f$A^T diag(w) A\f$, and an array `point_clusters` assigning each
	 * column of `A` to a cluster.  From this the array `out_group_inds` of
	 * length `num_clusters+1` is created such that `out_group_inds[i]`
	 * and `out_group_inds[i+1]` respectively beginning and end indices of
	 * the rows of `out_A` belonging to each cluster.  `out_A` and `out_w` are
	 * versions of `A` and `w` permuted to match the order implied by
	 * `out_group_inds`.
	 *
	 * Note that `A` and `w` must be created with their rows corresponding
	 * to the edges in the given `triangles` in the same order as defined
	 * by `cotangent_laplacian::mesh_laplacian`.
	 */
	static void apply_clusters_to_laplacian(
		SparseMatrix& out_A,
		Vector& out_w,
		IntVector& out_group_inds,
		const IntVector& point_clusters,
		const SparseMatrix& A,
		const Vector& w,
		const TriangleVector& triangles
	) {
		typedef typename SparseMatrix::InnerIterator InnerIterator;
		typedef typename Eigen::Triplet<Scalar> Triplet;
		Assert(point_clusters.size() == A.cols()) << "point_clusters_to_feature_groups requires `point_clusters.size() == A.cols()`" << raise;
		Assert(A.rows() == w.size()) << "point_clusters_to_feature_groups requires `A.rows() == w.size()`" << raise;
		Assert(A.rows() == 3*triangles.size()) << "rows of A must map to edges of triangles (in order)" << raise;

		Int num_clusters = point_clusters.maxCoeff() + 1;
		std::vector<std::vector<Int> > cluster_row_inds(num_clusters);

		// Map the point clusters to per-triangle clusters, which are then
		// mapped to the rows as `A`.  If the vertices to a triangle map to
		// different clusters, which one is chosen for the triangle is arbitrary.
		IntVector row_clusters = IntVector::Constant(A.rows(), -1);
		for(Int triangle_ind = 0; triangle_ind < triangles.size(); ++triangle_ind) {
			const Triangle& triangle = triangles[triangle_ind];
			Int cluster_ind = point_clusters[triangle[0]];

			Int i1, i2, i3;
			cotangent_laplacian<Scalar,2>::triangle_edge_indices(i1, i2, i3, triangle_ind);
			row_clusters[i1] = cluster_ind;
			row_clusters[i2] = cluster_ind;
			row_clusters[i3] = cluster_ind;
			cluster_row_inds[cluster_ind].push_back(i1);
			cluster_row_inds[cluster_ind].push_back(i2);
			cluster_row_inds[cluster_ind].push_back(i3);
		}

		// format `cluster_row_inds` into `out_group_inds` giving the start
		// and end indices of each cluster, and `row_permutation` describing
		// a premutation of the rows of `A and the elements of `w` needed to
		// match the ordering implied by `out_group_inds`.
		out_group_inds.resize(num_clusters+1);
		out_group_inds[0] = 0;
		Eigen::VectorXi row_permutation(A.rows()); // VectorXi instead of IntVector due to an Eigen 3.2 bug
		{
			Int row_ind = 0;
			for(Int cluster_ind = 0; cluster_ind < num_clusters; ++cluster_ind) {
				const std::vector<Int>& cluster_inds = cluster_row_inds.at(cluster_ind);
				for(Int i = 0; i < (Int)cluster_inds.size(); ++i) {
					row_permutation[cluster_inds[i]] = int_cast<int>(row_ind);
					++row_ind;
				}
				out_group_inds[cluster_ind+1] = row_ind;
			}
			assert(row_ind == A.rows());
		}

		// We'd like to be able to write the following code:
		//     auto P = row_permutation.matrix().asPermutation();
		//     out_A = P * A;
		//     out_w = P * w;
		// But due to a bug in certain versions of Eigen, it doesn't
		// compile with MSVC (depending on which version of Eigen
		// you're using).  So instead we have to manually perform the
		// permutation.
		build_matrices()
			.build(out_A, A.rows(), A.cols(), A.nonZeros())
			.build(out_w, w.size())
			.run(
		[&](auto& out_A, auto& out_w) {
			loop_nonzeros(A, [&](Int i, Int j, const Scalar& x) {
				Int i2 = row_permutation[i];
				out_A(i2, j) += x;
			});
			loop_nonzeros(w, [&](Int i, const Scalar& x) {
				Int i2 = row_permutation[i];
				out_w[i2] += x;
			});
		});
	}

	/**
	 * Clusters points by their Euciadean distance using a K-means algorithm.
	 * If desired, clusters smaller than a specified threshold will be
	 * automatically removed in a post-processing step.
	 */
	class KMeansClustersSolver {
		/**
		 * Denotes which state of the clustering to to be executed upon the next
		 * call to `iterate()`.
		 */
		enum class Stage {
			INIT,
			CLUSTER,
			REMOVE_SMALL_CLUSTERS,
			DONE
		};

	public:
		KMeansClustersSolver(
			const Matrix& P,
			Int num_clusters
		) :
			m_num_clusters(num_clusters),
			m_P(P),
			m_min_cluster_size(1),
			m_stage(Stage::INIT),
			m_point_clusters(P.cols()),
			m_cluster_centers(P.rows(), num_clusters),
			m_cluster_counts(num_clusters)
		{ }

		/**
		 * Sets the minimum allowed number of points in a cluster.  Clusters
		 * smaller than this will be removed in a post-processing step.  Note
		 * that this implies that the number of generated clusters may be
		 * smaller than the `num_clusters` parameter passed to the constructor.
		 * Set to zero or negative to disable.
		 */
		void set_min_cluster_size(Int min_size) {
			m_min_cluster_size = min_size;
		}

		Int dimension() const {
			return m_P.rows();
		}

		Int num_points() const {
			return m_P.cols();
		}

		Int num_clusters() const {
			return m_num_clusters;
		}

		const IntVector& point_clusters() const {
			return m_point_clusters;
		}

		bool iterate() {
			if(m_stage == Stage::INIT) {
				greedily_cluster_points(m_point_clusters, m_P, num_clusters());
				m_stage = Stage::CLUSTER;
				return true;
			} else if(m_stage == Stage::CLUSTER) {
				update_means(); // this is here bacuse it needs to be done first after `greedily_cluster_points`
				bool was_change = update_clusters();
				if(!was_change) {
					if(m_min_cluster_size > 0) {
						m_stage = Stage::REMOVE_SMALL_CLUSTERS;
					} else {
						m_stage = Stage::DONE;
					}
				}
			} else if(m_stage == Stage::REMOVE_SMALL_CLUSTERS) {
				bool was_change = remove_small_clusters();
				if(was_change) {
					update_clusters();
				} else {
					trim_removed_clusters();
					m_stage = Stage::DONE;
				}
			}
			return m_stage != Stage::DONE;
		}

		const IntVector& solve() {
			while(iterate());
			return point_clusters();
		}

	protected:
		void update_means() {
			assert(m_cluster_centers.rows() == m_P.rows());
			assert(m_cluster_centers.cols() == num_clusters());
			assert(m_cluster_counts.size() == num_clusters());
			m_cluster_centers.setZero();
			m_cluster_counts.setZero();
			for(Int point_ind = 0; point_ind < num_points(); ++point_ind) {
				Int cluster_ind = m_point_clusters[point_ind];
				m_cluster_centers.col(cluster_ind) += m_P.col(point_ind);
				m_cluster_counts[cluster_ind] += 1;
			}
			for(Int cluster_ind = 0; cluster_ind < num_clusters(); ++cluster_ind) {
				Int count = m_cluster_counts[cluster_ind];
				if(count > 0) {
					m_cluster_centers.col(cluster_ind) /= static_cast<Scalar>(count);
				}
			}
		}

		bool update_clusters() {
			bool was_change = false;
			for(Int point_ind = 0; point_ind < num_points(); ++point_ind) {
				auto p = m_P.col(point_ind);
				Int best_cluster_ind = -1;
				Scalar best_cluster_dist = std::numeric_limits<Scalar>::max();
				for(Int cluster_ind = 0; cluster_ind < num_clusters(); ++cluster_ind) {
					Scalar dist = (p - m_cluster_centers.col(cluster_ind)).squaredNorm();
					if(dist < best_cluster_dist) {
						best_cluster_ind = cluster_ind;
						best_cluster_dist = dist;
					}
				}

				assert(best_cluster_ind >= 0); // this can fail if m_P contains NaN or inf
				if(m_point_clusters[point_ind] != best_cluster_ind) {
					m_point_clusters[point_ind] = best_cluster_ind;
					was_change = true;
				}
			}
			return was_change;
		}

		/**
		 * Marks any small clusters so that they will be removed by a following
		 * call to `update_means()`,  Returns `true` if any clusters were
		 * actually marked for removal, and `false` otherwise.
		 */
		bool remove_small_clusters() {
			assert(m_min_cluster_size > 0); // otherwise this method shouldn't be called in the first place
			bool was_change = false;
			m_cluster_counts.setZero();
			for(Int point_ind = 0; point_ind < num_points(); ++point_ind) {
				Int cluster_ind = m_point_clusters[point_ind];
				m_cluster_counts[cluster_ind] += 1;
			}
			for(Int cluster_ind = 0; cluster_ind < num_clusters(); ++cluster_ind) {
				Int count = m_cluster_counts[cluster_ind];
				if(count > 0 && count < m_min_cluster_size) {
					m_cluster_centers.col(cluster_ind).setConstant(std::numeric_limits<Scalar>::max());
					was_change = true;
				}
			}
			return was_change;
		}

		/**
		 * Remaps the cluster indices so that any size-zero clusters are removed and
		 * the remaining clusters have indices which are consecutive integers
		 * starting at 0.
		 */
		void trim_removed_clusters() {
			Int old_num_clusters = m_num_clusters;
			IntVector old_point_clusters = m_point_clusters;
			Matrix old_cluster_centers = m_cluster_centers;
			IntVector old_cluster_counts = m_cluster_counts;

			// map old cluster IDs to new cluster IDs so the clusters remain numbered
			// by consecutive ints starting at 0
			IntVector cluster_map(old_num_clusters);
			cluster_map.setConstant(-1);
			Int num_clusters = 0;
			for(Int old_cluster_ind = 0; old_cluster_ind < old_num_clusters; ++old_cluster_ind) {
				if(old_cluster_counts[old_cluster_ind] > 0) {
					cluster_map[old_cluster_ind] = num_clusters;
					++num_clusters;
				}
			}

			// set the state of `this` to remove any size-zero clusters 
			m_num_clusters = num_clusters;
			m_cluster_centers.resize(m_cluster_centers.rows(), m_num_clusters);
			m_cluster_counts.resize(m_num_clusters);
			for(Int point_ind = 0; point_ind < num_points(); ++point_ind) {
				Int cluster_ind = cluster_map[old_point_clusters[point_ind]];
				assert(cluster_ind >= 0);
				m_point_clusters[point_ind] = cluster_ind;
			}
			for(Int cluster_ind = 0; cluster_ind < m_num_clusters; ++cluster_ind) {
				m_cluster_centers.col(cluster_ind) = old_cluster_centers.col(cluster_ind);
				m_cluster_counts[cluster_ind] = old_cluster_counts[cluster_ind];
			}
		}

	private:
		Int m_num_clusters;
		Matrix m_P;
		Int m_min_cluster_size;

		Stage m_stage;
		IntVector m_point_clusters;
		Matrix m_cluster_centers; // (dimension x num_clusters)
		IntVector m_cluster_counts;
	};
};


END_OPTIMTOOLS_NAMESPACE
