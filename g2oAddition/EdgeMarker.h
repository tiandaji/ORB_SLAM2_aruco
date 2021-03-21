/**
 * Add by liujiamin 2020-12-25
 * TODO: in g2o
 */

#ifndef ORB_SLAM2_EDGEMARKER_H
#define ORB_SLAM2_EDGEMARKER_H

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/hyper_graph_action.h"
#include "Thirdparty/g2o/g2o/core/eigen_types.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/stuff/misc.h"
#include "Thirdparty/g2o/g2o/types/se3quat.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"
#include "include/MapAruco.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace g2o
{

typedef Eigen::Matrix<double,8,1,Eigen::ColMajor>    Vector8D;

class EdgeMarker: public g2o::BaseBinaryEdge<8, Vector8D, VertexSE3Expmap, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeMarker(double length){
        auto po = ORB_SLAM2::MapAruco::get3DPointsLocalRefSystem(length);
        for(int i=0; i<4; i++)
        {
            auto &p = po[i];
            points[i] = g2o::Vector3D(p.x, p.y, p.z);
        }
    }

    virtual bool read(std::istream& is);

    virtual bool write(std::ostream& os) const;

    void computeError() {
        //marker
        const VertexSE3Expmap* g2m = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        //camera
        const VertexSE3Expmap* c2g = static_cast<const VertexSE3Expmap*>(_vertices[1]);

        //step 1: get points in camera reference
        auto T_c2m = c2g->estimate() * g2m->estimate();
        g2o::Vector3D points2[4];
        for(int i=0; i<4; i++)
        {
            points2[i]=T_c2m.map(points[i]);
        }
        //step 2: error
        _error.resize(8);
        Vector8D obs(_measurement);
        assert(_error.size()==obs.size());
        int idx=0;
        for(int i=0; i<4; i++)
        {
            double projx=( points2[i](0)/points2[i](2)) *fx +cx;
            _error(idx)=obs(idx)-projx;
            idx++;
            double projy=( points2[i](1)/points2[i](2)) *fy +cy;
            _error(idx)=obs(idx)-projy;
            idx++;
        }

    }

    g2o::Vector3D points[4];
    // uint32_t marker_id, frame_id;
    double fx, fy, cx, cy;

};


} // namespace g2o


#endif //ORB_SLAM2_EDGEMARKER_H