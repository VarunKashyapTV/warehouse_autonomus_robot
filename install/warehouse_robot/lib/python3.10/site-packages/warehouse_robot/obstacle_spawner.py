import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity
import math

# every value hardcoded is in meter per seconds or meters


class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        self.subscription_1 = self.create_subscription(Path, "/plan", self.plan_callback, 10)

        self.subscription_2 = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')

        self.cli = self.create_client(SpawnEntity, '/spawn_entity')

        self.lane_length =1

        self.spwan_buffer = 7

        self.spawned_obstacle_names = []

        self.fractions = [0.25, 0.5, 0.75]

        self.spawned_cooredinates = []

        self.current_path_list = []

        self.current_x,self.current_y = 0,0

        self.count = 0

        self.checkpoints_list = []

        self.temp = 1

        self.dest_x,self.dest_y = 0,0

    def Update_checkpoints(self,poses):
         self.checkpoints_list.clear()
         if self.count == 3:
              self.checkpoints_list.append(poses[int(0.25*len(poses))])
              self.checkpoints_list.append(poses[int(0.50*len(poses))])
              self.checkpoints_list.append(poses[int(0.75*len(poses))])
              self.temp=len(self.checkpoints_list)
              return self.checkpoints_list

         elif self.count == 2:
              self.checkpoints_list.append(poses[int(0.50*len(poses))])
              self.checkpoints_list.append(poses[int(0.75*len(poses))])
              self.temp=len(self.checkpoints_list)
              return self.checkpoints_list

         elif self.count == 1:
              self.checkpoints_list.append(poses[int(0.50*len(poses))])
              self.temp=len(self.checkpoints_list)
              return self.checkpoints_list
         
         else:
              self.temp=len(self.checkpoints_list)
              return self.checkpoints_list
              

    def plan_callback(self, msg: Path):
        new_dest_x = msg.poses[-1].pose.position.x

        new_dest_y = msg.poses[-1].pose.position.y
    
        if (round(new_dest_x, 2), round(new_dest_y, 2)) == (round(self.dest_x, 2), round(self.dest_y, 2)):
                self.current_path_list = msg.poses
        else: 
                self.dest_x,self.dest_y = new_dest_x,new_dest_y
                self.path_length = self.compute_path_length(msg.poses)
                if  self.path_length < 1*self.lane_length:
                     self.count=0

                elif self.path_length<=2*self.lane_length and self.path_length>=1*self.lane_length :
                     self.count=1

                elif self.path_length <=4*self.lane_length and self.path_length >=2*self.lane_length:
                     self.count=2

                elif self.path_length >4*self.lane_length :
                     self.count=3
               

                self.checkpoints_list = self.Update_checkpoints(msg.poses)
                self.current_path_list = msg.poses

               
            
    def compute_path_length(self,poses):
          sum = 0
          self.new_x,self.new_y=0,0
          for pose in poses:
                    if self.new_x == 0 and self.new_y == 0:
                        self.new_x,self.new_y = pose.pose.position.x,pose.pose.position.y

                    else:
                        sum += math.hypot(self.new_x - pose.pose.position.x,self.new_y - pose.pose.position.y)
                        self.new_x,self.new_y = pose.pose.position.x,pose.pose.position.y
          return sum

    def odom_callback(self, msg: Odometry):
          self.current_x = msg.pose.pose.position.x
          self.current_y = msg.pose.pose.position.y

          if not self.checkpoints_list:
           return 
          elif  math.hypot(self.current_x-self.checkpoints_list[0].pose.position.x,self.current_y-self.checkpoints_list[0].pose.position.y)<=0.4:
           self.trigger_spawn(self.checkpoints_list[0].pose.position.x,self.checkpoints_list[0].pose.position.y)    


    def trigger_spawn(self,x,y):
         self.count-=1

         self.spawn_obstacle(x,y)

         self.checkpoints_list.pop(0)

         self.checkpoints_list=self.Update_checkpoints(self.current_path_list)




    def spawn_obstacle(self, x, y):
        # build the SDF string, build the service request, call it
        self.box_name = f"obstacle_box {self.count}"
        self.spawned_obstacle_names.append(self.box_name)
        self.sdf = f"""<sdf version="1.6">
                    <model name="{self.box_name}">
                    <static>true</static>
                    <link name="link">
                         <collision name="collision">
                         <geometry>
                              <box><size>0.5 0.5 1.0</size></box>
                         </geometry>
                         </collision>
                         <visual name="visual">
                         <geometry>
                              <box><size>0.5 0.5 1.0</size></box>
                         </geometry>
                         </visual>
                    </link>
                    </model>
                    </sdf>"""
        self.req = SpawnEntity.Request()
        self.req.name = self.box_name
        self.req.xml = self.sdf
        self.req.initial_pose.position.x = x
        self.req.initial_pose.position.y = y
        self.spawned_cooredinates.append([x,y])
        self.req.reference_frame = "map"

        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.spawn_response)

        if len(self.spawned_obstacle_names) > self.spwan_buffer:
             self.delete_req = DeleteEntity.Request()
             self.delete_req.name = self.spawned_obstacle_names[0]
             self.spawned_obstacle_names.pop(0)
             self.del_future = self.delete_cli.call_async(self.delete_req)
             self.del_future.add_done_callback(self.del_response)




             

    def del_response(self,future):
         self.get_logger().info(f"{future.result().success}")
         self.get_logger().info(f"{future.result().status_message}")
         pass    
    def spawn_response(self,future):
         
     self.get_logger().info(f"{future.result().success}")
     self.get_logger().info(f"{future.result().status_message}")

     pass
    


def main():
    rclpy.init()
    node = ObstacleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()