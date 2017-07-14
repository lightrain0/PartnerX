#ifndef AREA_TOOL_H_
#define AREA_TOOL_H_

#ifndef DEPRECATED
#if defined(__GNUC__)
#define DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#else 
#define DEPRECATED
#endif
#endif

#include "ros/ros.h"
#include "cleaner_pathplan/space_type.h"
using namespace space_type;

namespace area_tool{

    const bool COVER_DIRECTION_CW   =  false; 

    bool GetDirectionCW(TDirection& dir)
    {
        // X_N > Y_N > X_P > Y_P 
        TDirection::DirEnum prev_dir = TDirection::NONE;
        switch(dir.value)
        {
            case TDirection::X_N : 
                prev_dir = TDirection::Y_P;
            break;
            case TDirection::Y_N : 
                prev_dir = TDirection::X_N;
            break;
            case TDirection::X_P : 
                prev_dir = TDirection::Y_N;
            break;
            case TDirection::Y_P : 
                prev_dir = TDirection::Y_P;
            break;
            default:
            ROS_ERROR("WRONG PARAM WHEN GETTING PREVIOUS DIRECTION");
            return false;
            break;
        }
        dir.value  = prev_dir;
        return true;
    }
    bool GetDirectionCCW(TDirection& dir)
    {
        // X_N > Y_N > X_P > Y_P 
        TDirection::DirEnum next_dir = TDirection::NONE;
        switch(dir.value)
        {
            case TDirection::X_N : 
                next_dir = TDirection::Y_N;
            break;
            case TDirection::Y_N : 
                next_dir = TDirection::X_P;
            break;
            case TDirection::X_P : 
                next_dir = TDirection::Y_P;
            break;
            case TDirection::Y_P : 
                next_dir = TDirection::X_N;
            break;
            default:
            ROS_ERROR("WRONG PARAM WHEN GETTING NEXT DIRECTION");
            return false;
            break;
        }
        dir.value  = next_dir;
        return true;
    }
    bool GetPreviousDirection( TDirection& dir)  //RECT 
    {
        if(COVER_DIRECTION_CW)
            return GetDirectionCCW(dir);
        else
            return GetDirectionCW(dir);
    }
    bool GetNextDirection( TDirection& dir) //RECT 
    {
        if(COVER_DIRECTION_CW)
            return GetDirectionCW(dir);
        else
            return GetDirectionCCW(dir);
    }
     bool  CreateArea(TArea& area, const TPose& first_pose,const TDirection& first_dir) //AREA
    {
        TDirection dir = first_dir;
        TPose pose = first_pose;
        TArea new_area;
        // Initialize area state
        new_area.state.value = TAreaState::NONE;
        
        // Create rect  
        TRect& rect = new_area.shape;
        rect.width = 1;
        rect.height = 1;
        TDirection prev_dir = dir;
        if(!GetPreviousDirection(prev_dir))
            return false;
        for(int i ; i < rect.joint_count;i ++)
        {
            TJoint joint ;
            joint.pose = pose;
            joint.dir_output = dir;
            joint.dir_input = prev_dir;
            // 
            rect.joints.push_back(joint);

            // Prepare next positoin and direction 
            switch(dir.value)
            {
                case TDirection::X_N:
                    pose.x -= rect.width;
                break;
                case TDirection::Y_N:
                    pose.y -= rect.width;
                break;
                case TDirection::X_P:
                    pose.x += rect.width;
                break;
                case TDirection::Y_P:
                    pose.y += rect.width;
                break;
            }
            //

            prev_dir = dir;
            if(!GetNextDirection(dir))
                return false;

        }
        //Add the first pose as the first node
        TNode node;
        node.pose = first_pose;
        TNode::NodeProperty node_prop;
        node_prop.property = TNode::NodeProperty::PROP_Jo;
        node_prop.direction = first_dir;
        node_prop.state = TNode::NodeProperty::TRANSMITTED;

        node.properties.push_back(node_prop);

        new_area.nodes.push_back(node);
        area = new_area;
        return true;
    }
    bool  CreateArea(TArea& area, const TNode& node) //AREA
    {
        TDirection first_dir;
        first_dir.value = TDirection::NONE;
        for(int i = 0; i < node.properties.size() ;i ++ )
        {
            TNode::NodeProperty prop = node.properties.at(i);
            if( (prop.property & TNode::NodeProperty::PROP_Jo) && prop.state == TNode::NodeProperty::TRANSMITTED)
            {
                //  Find the highest priority one 
                if((int)first_dir.value < (int)prop.direction.value)
                    first_dir = prop.direction;
            }
        }
        if(first_dir.value != TDirection::NONE)
            return CreateArea(area,node.pose,first_dir);
        else
          {
            ROS_WARN("The param node is not available to create an area");
            return false;
          } 
    }
    bool AddAreaToRoom(TRoom& room,const TArea& area) // ROOM
    {
        room.areas.push_back(area);
        return true;
    }
    bool AddPropertyToNode(TNode& node,TNode::NodeProperties node_props) // NODE 
    {
        if(node.properties.size() == 0)
            node.properties.insert(node.properties.end(),node_props.begin(),node_props.end());
        else
        {
            for(int j = 0; j < node_props.size(); j ++)
            {
                TNode::NodeProperty node_prop = node_props.at(j);
                TDirection dir;
                dir.value = node_prop.direction.value;
                for(int i = 0;i < node.properties.size();i ++)
                {
                    TNode::NodeProperty& prop = node.properties.at(i);
                    if(dir.value == prop.direction.value)
                    {
                        if(prop.state != node_prop.state)
                        {
                            prop.state = node_prop.state = TNode::NodeProperty::BALLANCED;
                        }
                    }
                }
                node.properties.push_back(node_prop);
            }
            
        }
        return true;
    }
    bool AddNodeToArea(TArea& area,const TNode& new_node) //AREA 
    {
        for(int i = 0;i < area.nodes.size();i ++)
        {
            TNode& node = area.nodes.at(i);
            // This equality judgement should be improved for position tolerance
            if(new_node.pose.x == node.pose.x && new_node.pose.y == node.pose.y)
             {

                AddPropertyToNode(node,new_node.properties);
                return true;
             }
        }
        area.nodes.push_back(new_node);
        return true;
    }
    bool CreateNode(TNode& node,const TPose& cur_pose,TNode::NodeProperty *node_prop) // NODE
    {
        node.pose = cur_pose;
        if(node_prop)
            node.properties.push_back(*node_prop);
        return true;
    }
    
    bool IsCoveredOff(const TArea& area) //AREA
    {
        return area.cur_line == area.shape.joint_count;
    }
    bool GetNextLinePosition(TArea& area,TPose& pose) //AREA 
    {
        if(IsCoveredOff(area))
        {
            return false;
        }

        ++area.cur_line;
        if(area.cur_line == area.shape.joint_count)
            pose  =  area.shape.joints.at(0).pose;
        else
            pose  =  area.shape.joints.at(area.cur_line).pose;
        
        return true;
    }
    bool GetCurrentLinePosition(TArea& area,TPose& pose) //AREA 
    {
        if(area.cur_line == area.shape.joint_count)
            pose  =  area.shape.joints.at(0).pose;
        else
            pose  =  area.shape.joints.at(area.cur_line).pose;
        return true;
    }
    bool GetMin(const TRect& shape,TPose::_x_type& min_x,TPose::_y_type& min_y,TPose::_z_type& min_z) // SHAPE
    {
        min_x = min_y = min_z = 10000;
        for(int i = 0;i < shape.joints.size();i ++)
        {
            if(shape.joints.at(i).pose.x < min_x)
                min_x = shape.joints.at(i).pose.x;
            if(shape.joints.at(i).pose.y < min_y)
                min_y = shape.joints.at(i).pose.y;
            if(shape.joints.at(i).pose.z < min_z)
                min_z = shape.joints.at(i).pose.z;
        }
        return true;
    }
    bool GetMax(const TRect& shape,TPose::_x_type& max_x,TPose::_y_type& max_y,TPose::_z_type& max_z) // SHAPE
    {
        max_x = max_y = max_z = -10000;
        for(int i = 0;i < shape.joints.size();i ++)
        {
            if(shape.joints.at(i).pose.x > max_x)
                max_x = shape.joints.at(i).pose.x;
            if(shape.joints.at(i).pose.y > max_y)
                max_y = shape.joints.at(i).pose.y;
            if(shape.joints.at(i).pose.z > max_z)
                max_z = shape.joints.at(i).pose.z;
        }
        return true;
    }
    bool IsBeyondBoundary(const TArea& area,const TPose& cur_pose) // AREA to find Bo
    {
        TPose::_x_type min_x,max_x;
        TPose::_y_type min_y,max_y;
        TPose::_z_type min_z,max_z;
        GetMin(area.shape,min_x,min_y,min_z);
        GetMax(area.shape,max_x,max_y,max_z);

        if(cur_pose.x <= min_x || cur_pose.y <= min_y || cur_pose.x >= max_x || cur_pose.y >= max_y)
            return true;
        return false;
    }
    bool IsOverLine(const TArea& area,const TPose& cur_pose) // AREA  to find Jo 
    {
        TJoint dest_joint = area.shape.joints.at(area.cur_line);
        const TPose& dest_pose = dest_joint.pose; 
        switch(dest_joint.dir_input.value)
        {
            case TDirection::X_N:
                return dest_pose.x >= cur_pose.x; 
                break;
            case TDirection::Y_N:
                return dest_pose.y >= cur_pose.y; 
                break;
            case TDirection::X_P:
                return dest_pose.x <= cur_pose.x; 
                break;
            case TDirection::Y_P:
                return dest_pose.y <= cur_pose.y;
                break;
            default:
                ROS_ERROR("WRONG JOINT IN THE AREA SHAPE , PLEASE CHECK ITS DIRECTION");
            break;
        }
    }
    
    bool FindActiveNode(const TRoom& room,TNode& node) // ROOM to Find Jo Node
    {
        for(int n = 0;n < room.areas.size(); n ++)
        {
            for(int i = 0;i < room.areas.at(n).nodes.size();i ++)
            {
                const TNode& n_node = room.areas.at(n).nodes.at(i);
                for(int j = 0; j < n_node.properties.size();j ++)
                {
                    const TNode::NodeProperty& prop = n_node.properties.at(i);
                    if(prop.property == TNode::NodeProperty::PROP_Jo && prop.state == TNode::NodeProperty::RECEIVED)
                    {
                        node = n_node;
                        return true;     
                    }
                }
            
            }
        }
        return false;
    }
    bool IsRoomCovered(const TRoom& room) // ROOM
    {
        for(int n = 0;n < room.areas.size(); n ++)
        {
            for(int i = 0;i < room.areas.at(n).nodes.size();i ++)
            {
                const TNode& n_node = room.areas.at(n).nodes.at(i);
                for(int j = 0; j < n_node.properties.size();j ++)
                {
                    const TNode::NodeProperty& prop = n_node.properties.at(i);
                    if(prop.property == TNode::NodeProperty::PROP_Jo && prop.state != TNode::NodeProperty::BALLANCED)
                    {
                        return false;     
                    }
                }
            }
        }
        return true;
    }
    bool IsOccupied(const TRoom& room,const TPose& pose) // ROOM 
    {
         for(int i = 0;i < room.areas.size();i ++)
        {
            if(!IsBeyondBoundary(room.areas.at(i),pose))
                return true;
        }
        return false;
    }
}


#endif

