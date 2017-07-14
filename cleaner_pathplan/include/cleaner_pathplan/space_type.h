#ifndef SPACE_TYPE_H_
#define SPACE_TYPE_H_

#ifndef DEPRECATED
#if defined(__GNUC__)
#define DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#else 
#define DEPRECATED
#endif
#endif

#include <string>
#include <vector>
namespace space_type{
    //! \brief Define a struct to maintain position in the local coordinate
    struct TPose
    {
        typedef double _x_type;
        _x_type x;
        typedef double _y_type;
        _y_type y;
        typedef double _z_type;
        _z_type z;

        double roll,pitch,yaw;
    };
    //
    struct TDirection
    {
        // oder with priority , bigger the number , heigher the priority 
        enum DirEnum{
            NONE = 0,
            X_N = 4,
            Y_N = 3,
            X_P = 2,
            Y_P = 1
        };
        DirEnum value;
    };

    //
    template <class Direction,typename Property> 
    struct TNodeProperty_
    {
        enum PropertyEnum{
            PROP_Jo = 0x01,
            PROP_Bo = 0x02,
            PROP_Co = 0x03,
            PROP_Ob = 0x04,
            PROP_Oc = 0x05
        };
        enum PropStateEnum{
            TRANSMITTED,
            RECEIVED,
            BALLANCED
        };
        PropertyEnum        property;
        Direction       direction;
        PropStateEnum      state;
    };
    //
    template <class JointSpec> 
    struct  TNode_
    {
         
        typedef typename JointSpec::_pose_type PoseType;
        typedef typename JointSpec::_dir_type  DirType;
        typedef int  PropType;
        typedef TNodeProperty_<DirType,PropType> NodeProperty;
        typedef std::vector< NodeProperty>  NodeProperties;

        PoseType pose;
        std::vector< NodeProperty> properties;
         
        
    };
    //
    template <class JointSpec>
    struct TRect_
    {
        typedef JointSpec   _joint_type;
        std::vector<_joint_type> joints; 

        typedef double     _stride_type;
        _stride_type       width,height; //By default, width along with x axis, height along with y axis;
        static const int    joint_count = 4;
    };
    
    //
    template <class PoseSpec,class Direction>
    struct TJoint_
    {
        typedef PoseSpec _pose_type;
        typedef Direction _dir_type;
        //
        _dir_type  dir_input;
        _dir_type  dir_output;
        _pose_type  pose;
    };
    //
        //
    struct TAreaState
    {
        enum AreaStateEnum{
            NONE,
            PENDING,
            DONE
        };
        AreaStateEnum   value;
    };
    //
    template <class ShapeSpec ,class NodeSpec>
    struct TArea_ 
    {
        typedef NodeSpec _node_type;
        typedef ShapeSpec _shape_type;
        

        _shape_type   shape;
        std::vector<_node_type> nodes;
        TAreaState    state;
        int           cur_line; // based on zero , one is the first line to move , joint_count of shape is to the end of rect 
       
    };
    //
    template <class AreaSpec>
    struct TRoom_ 
    {
        typedef typename AreaSpec::_node_type _node_type;
        std::vector<AreaSpec> areas;
    //    std::vector<_node_type> node_queue;
    };
    //
    typedef  TJoint_<TPose,TDirection>            TJoint;
    typedef  TNode_<TJoint >                      TNode;
    typedef  TRect_<TJoint >                      TRect;
    typedef  TArea_<TRect , TNode>                TArea;
    typedef  TRoom_<TArea>                        TRoom;
}

#undef DEPRECATED
#endif
