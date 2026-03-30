/*************************************************************************
 *
 * ADOBE CONFIDENTIAL
 * ___________________
 *
 *  Copyright 2025 Adobe Systems Incorporated
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of Adobe Systems Incorporated and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to Adobe Systems Incorporated and its
 * suppliers and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Adobe Systems Incorporated.
 **************************************************************************/

#ifndef CORE_UTIL_NEARESTNEIGHBORSEARCH_H_
#define CORE_UTIL_NEARESTNEIGHBORSEARCH_H_

#include <vector>

#include "../core-base/common.h"

namespace ml
{

template<class FloatType>
class NearestNeighborSearch
{
public:
    void init(const std::vector< const FloatType* > &points, uint32_t dimension, uint32_t maxK)
    {
        initInternal(points, dimension, maxK);
    }
    void kNearest(const FloatType *query, uint32_t k, FloatType epsilon, std::vector<uint32_t> &result) const
    {
        kNearestInternal(query, k, epsilon, result);
    }
    uint32_t nearest(const FloatType *query) const
    {
        std::vector<uint32_t> result;
        kNearestInternal(query, 1, 0.0f, result);
        return result[0];
    }
    
    void init(const std::vector< std::vector<FloatType> > &points, uint32_t maxK)
    {
        //Vector< const D* > v = points.map([](const Vector<D> &x) {return x.ptr();});
        std::vector< const FloatType* > v(points.size());
        for(uint32_t i = 0; i < points.size(); i++)
            v[i] = &points[i][0];
        init(v, (uint32_t)points[0].size(), maxK);
    }

    //! init from a linearized array (needs an additional copy)
    void init(const FloatType* points, uint32_t numPoints, uint32_t dimension, uint32_t maxK)
    {
        std::vector<const FloatType*> pointsPtr(numPoints);

        for (size_t i = 0; i < pointsPtr.size(); i++) {
            pointsPtr[i] = &points[i*dimension];
        }
        init(pointsPtr, dimension, maxK);
    }

    void kNearest(const std::vector<FloatType> &query, uint32_t k, FloatType epsilon, std::vector<uint32_t> &result) const
    {
        kNearestInternal(&query[0], k, epsilon, result);
    }

    std::vector<uint32_t> kNearest(const std::vector<FloatType> &query, uint32_t k, FloatType epsilon) const
    {
        std::vector<uint32_t> result;
        kNearestInternal(&query[0], k, epsilon, result);
        return result;
    }

    std::vector<uint32_t> kNearest(const FloatType *query, uint32_t k, FloatType epsilon) const
    {
        std::vector<uint32_t> result;
        kNearestInternal(query, k, epsilon, result);
        return result;
    }

    std::vector<uint32_t> fixedRadius(const std::vector<FloatType> &query, uint32_t k, FloatType radiusSq) const
    {
        std::vector<uint32_t> result;
        fixedRadiusInternal(&query[0], k, radiusSq, 0.0f, result);
        return result;
    }

    std::vector<uint32_t> fixedRadius(const FloatType *query, uint32_t k, FloatType radiusSq) const
    {
        std::vector<uint32_t> result;
        fixedRadiusInternal(query, k, radiusSq, 0.0f, result);
        return result;
    }

    std::vector< std::pair<uint32_t, FloatType> > fixedRadiusDist(const FloatType *query, uint32_t k, FloatType radiusSq) const
    {
        std::vector< std::pair<uint32_t, FloatType> > result;
        fixedRadiusInternalDist(query, k, radiusSq, 0.0f, result);
        return result;
    }

    virtual void getDistances(unsigned int k, std::vector<FloatType> &dists) const
    {
        throw MLIB_EXCEPTION("kNearest not implemented");
    }

private:
    virtual void initInternal(const std::vector< const FloatType* > &points, uint32_t dimension, uint32_t maxK) = 0;
    virtual void kNearestInternal(const FloatType *query, uint32_t k, FloatType epsilon, std::vector<uint32_t> &result) const = 0;
    virtual void fixedRadiusInternal(const FloatType *query, uint32_t k, FloatType radiusSq, FloatType epsilon, std::vector<uint32_t> &result) const = 0;
    virtual void fixedRadiusInternalDist(const FloatType *query, uint32_t k, FloatType radiusSq, FloatType epsilon, std::vector< std::pair<uint32_t, FloatType> > &result) const = 0;
};

template<class FloatType>
class KNearestNeighborQueue
{
public:
    KNearestNeighborQueue() {}

    struct NeighborEntry
    {
        NeighborEntry() {}
        NeighborEntry(int _index, FloatType _dist)
        {
            index = _index;
            dist = _dist;
        }
        int index;
        FloatType dist;
    };

    KNearestNeighborQueue(uint32_t k, FloatType clearValue)
    {
        init(k, clearValue);
    }

    void init(uint32_t k, FloatType clearValue)
    {
        if(m_queue.size() != k) m_queue.resize(k);
        clear(clearValue);
    }

    void clear(FloatType clearValue)
    {
        m_queue.assign(m_queue.size(), NeighborEntry(-1, clearValue));
        m_farthestDist = clearValue;
    }

    inline void insert(int index, FloatType dist)
    {
        insert(NeighborEntry(index, dist));
    }

    inline void insert(const NeighborEntry &entry)
    {
        if(entry.dist < m_farthestDist)
        {
            m_queue.back() = entry;
            const int queueLength = (int)m_queue.size();
            if(queueLength > 1)
            {
                NeighborEntry *data = &m_queue[0];
                for(int index = queueLength - 2; index >= 0; index--)
                {
                    if(data[index].dist > data[index + 1].dist)
                    {
                        std::swap(data[index], data[index + 1]);
                    }
                }
            }
            m_farthestDist = m_queue.back().dist;
        }
    }

    const std::vector<NeighborEntry>& queue() const
    {
        return m_queue;
    }

private:
    FloatType m_farthestDist;
    std::vector<NeighborEntry> m_queue;
};

template<class FloatType>
class NearestNeighborSearchBruteForce : public NearestNeighborSearch<FloatType>
{
public:
    NearestNeighborSearchBruteForce() {}

    void initInternal(const std::vector< const FloatType* > &points, uint32_t dimension, uint32_t maxK)
    {
        m_dimension = dimension;
        m_pointData.resize(points.size() * dimension);
        int pointIndex = 0;
        for (auto p : points)
        {
            m_points.push_back(&m_pointData[0] + pointIndex);
            for(uint32_t d = 0; d < m_dimension; d++)
                m_pointData[pointIndex++] = p[d];
        }
    }

    void kNearestInternal(const FloatType *query, uint32_t k, FloatType epsilon, std::vector<uint32_t> &result) const
    {
        m_queue.init(k, std::numeric_limits<FloatType>::max());
        
        for(uint32_t pointIndex = 0; pointIndex < m_points.size(); pointIndex++)
        {
            FloatType dist = 0.0f;
            const FloatType* point = m_points[pointIndex];
            for(uint32_t d = 0; d < m_dimension; d++)
            {
                FloatType diff = point[d] - query[d];
                dist += diff * diff;
            }
            m_queue.insert(typename KNearestNeighborQueue<FloatType>::NeighborEntry(pointIndex, dist));
        }

        if (result.size() != k) result.resize(k);
        uint32_t resultIndex = 0;
        for (const auto &e : m_queue.queue())
        {
            result[resultIndex++] = e.index;
        }
    }

    void fixedRadiusInternal(const FloatType *query, uint32_t k, FloatType radiusSq, FloatType epsilon, std::vector<uint32_t> &result) const
    {
        throw MLIB_EXCEPTION("fixedRadiusInternal not implemented");
    }

    void fixedRadiusInternalDist(const FloatType *query, uint32_t k, FloatType radiusSq, FloatType epsilon, std::vector< std::pair<uint32_t, FloatType> > &result) const
    {
        throw MLIB_EXCEPTION("fixedRadiusInternalDist not implemented");
    }

private:
    uint32_t m_dimension;
    std::vector<FloatType> m_pointData;
    std::vector< const FloatType* > m_points;
    mutable KNearestNeighborQueue<FloatType> m_queue;
};

}  // namespace ml

#endif  // CORE_UTIL_NEARESTNEIGHBORSEARCH_H_
