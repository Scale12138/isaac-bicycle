from omni.isaac.kit import SimulationApp  
import carb  
import numpy as np  
from pxr import Usd, UsdPhysics, Gf, Sdf  

class JointTester:  
    def __init__(self, usd_path: str, root_prim_path: str = "/World/Robot"):  
        """  
        USD关节测试工具  
        :param usd_path: USD模型文件路径  
        :param root_prim_path: 模型根路径  
        """  
        self._stage = None  
        self._joints_info = {}  
        self._current_joint = None  
        self._root_prim_path = root_prim_path  
        
        # 初始化场景  
        self._load_usd_model(usd_path)  
        self._detect_joints()  
        
    def _load_usd_model(self, usd_path: str):  
        """加载USD模型并设置物理属性"""  
        self._stage = omni.usd.get_context().open_stage(usd_path)  
        UsdPhysics.Scene.Define(self._stage, Sdf.Path("/physicsScene"))  
        self._stage.SetDefaultPrim(self._stage.GetPrimAtPath(self._root_prim_path))  
        
        # 配置物理场景  
        physicsScene = UsdPhysics.Scene.Get(self._stage, "/physicsScene")  
        physicsScene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))  
        physicsScene.CreateGravityMagnitudeAttr().Set(9.81)  

    def _detect_joints(self):  
        """自动检测所有关节并分类"""  
        prim = self._stage.GetPrimAtPath(self._root_prim_path)  
        for p in prim.GetChildren():  
            if p.GetTypeName() == "PhysicsRevoluteJoint":  
                joint_type = "revolute"  
                limit_attr = "upperLimit"  
            elif p.GetTypeName() == "PhysicsPrismaticJoint":  
                joint_type = "prismatic"  
                limit_attr = "upperLimit"  
            else:  
                continue  

            joint_path = str(p.GetPath())  
            drive_api = UsdPhysics.DriveAPI.Get(self._stage, joint_path, "angular")  
            if not drive_api:  
                drive_api = UsdPhysics.DriveAPI.Get(self._stage, joint_path, "linear")  
            
            self._joints_info[joint_path] = {  
                "type": joint_type,  
                "drive": drive_api,  
                "limits": {  
                    "lower": p.GetAttribute(f"physics:{limit_attr}").Get(),  
                    "upper": p.GetAttribute(f"physics:{limit_attr}").Get()  
                },  
                "current_pos": 0.0,  
                "current_vel": 0.0  
            }  

    def list_joints(self):  
        """列出所有可用关节"""  
        print("\n=== 检测到的关节列表 ===")  
        for i, (path, info) in enumerate(self._joints_info.items()):  
            print(f"[{i+1}] {path}")  
            print(f"   类型: {info['type'].upper()}")  
            print(f"   驱动模式: {self._get_drive_mode(info['drive'])}")  
            print(f"   位置限制: {info['limits']['lower']} - {info['limits']['upper']}\n")  

    def select_joint(self, index: int):  
        """选择当前操作的关节"""  
        joints = list(self._joints_info.keys())  
        if index < 0 or index >= len(joints):  
            raise ValueError("无效的关节索引")  
        self._current_joint = joints[index]  
        print(f"已选择关节: {self._current_joint}")  

    def configure_drive(self, drive_mode: str, max_force: float = 1000.0, damping: float = 50.0, stiffness: float = 0.0):  
        """  
        配置关节驱动参数  
        :param drive_mode: 驱动模式 (position/velocity/force)  
        :param max_force: 最大驱动力(N或N·m)  
        :param damping: 阻尼系数  
        :param stiffness: 刚度系数  
        """  
        if not self._current_joint:  
            raise RuntimeError("请先选择关节")  
            
        drive_api = self._joints_info[self._current_joint]["drive"]  
        drive_type = "angular" if self._joints_info[self._current_joint]["type"] == "revolute" else "linear"  
        
        # 设置驱动模式  
        if drive_mode == "position":  
            drive_api.CreateTypeAttr("acceleration")  
            drive_api.CreateMaxForceAttr(max_force)  
            drive_api.CreateDampingAttr(damping)  
            drive_api.CreateStiffnessAttr(stiffness)  
        elif drive_mode == "velocity":  
            drive_api.CreateTypeAttr("velocity")  
            drive_api.CreateMaxForceAttr(max_force)  
            drive_api.CreateDampingAttr(damping)  
            drive_api.CreateStiffnessAttr(0.0)  
        elif drive_mode == "force":  
            drive_api.CreateTypeAttr("force")  
            drive_api.CreateMaxForceAttr(max_force)  
        else:  
            raise ValueError("未知驱动模式")  

        print(f"{self._current_joint} 驱动模式设置为: {drive_mode.upper()}")  

    def send_command(self, target: float):  
        """  
        发送控制指令  
        :param target: 目标值（位置: rad/m, 速度: rad/s/m/s, 力: N/N·m）  
        """  
        if not self._current_joint:  
            raise RuntimeError("请先选择关节")  
            
        joint_info = self._joints_info[self._current_joint]  
        drive_api = joint_info["drive"]  
        
        if drive_api.GetTypeAttr().Get() == "acceleration":  
            drive_api.GetTargetPositionAttr().Set(target)  
        elif drive_api.GetTypeAttr().Get() == "velocity":  
            drive_api.GetTargetVelocityAttr().Set(target)  
        elif drive_api.GetTypeAttr().Get() == "force":  
            drive_api.GetTargetForceAttr().Set(target)  
            
        print(f"指令已发送: {target} {'rad' if joint_info['type']=='revolute' else 'm'}")  

    def monitor_joint(self, duration: float = 5.0):  
        """监控关节状态"""  
        from omni.isaac.core import SimulationContext  
        sim_context = SimulationContext()  
        
        print(f"\n监控关节 {self._current_joint}...")  
        print("时间(s)\t位置\t速度\t施加力")  
        
        start_time = sim_context.current_time  
        while sim_context.current_time - start_time < duration:  
            sim_context.step()  
            
            # 获取实时数据  
            pos = self._get_joint_position()  
            vel = self._get_joint_velocity()  
            force = self._get_applied_force()  
            
            print(f"{sim_context.current_time:.2f}\t{pos:.3f}\t{vel:.3f}\t{force:.1f}")  

    def _get_joint_position(self):  
        """获取当前关节位置"""  
        joint = UsdPhysics.RevoluteJoint.Get(self._stage, self._current_joint)  
        if joint:  
            return joint.GetAngleAttr().Get()  
        else:  
            prismatic = UsdPhysics.PrismaticJoint.Get(self._stage, self._current_joint)  
            return prismatic.GetPositionAttr().Get()  

    def _get_joint_velocity(self):  
        """获取当前关节速度"""  
        joint = UsdPhysics.RevoluteJoint.Get(self._stage, self._current_joint)  
        if joint:  
            return joint.GetVelocityAttr().Get()  
        else:  
            prismatic = UsdPhysics.PrismaticJoint.Get(self._stage, self._current_joint)  
            return prismatic.GetVelocityAttr().Get()  

    def _get_applied_force(self):  
        """获取当前施加力/力矩"""  
        drive_api = self._joints_info[self._current_joint]["drive"]  
        return drive_api.GetMaxForceAttr().Get()  

    def _get_drive_mode(self, drive_api):  
        """获取当前驱动模式"""  
        if drive_api.GetTypeAttr().Get() == "acceleration":  
            return "POSITION"  
        elif drive_api.GetTypeAttr().Get() == "velocity":  
            return "VELOCITY"  
        else:  
            return "FORCE"  

# 使用示例  
if __name__ == "__main__":  
    # 初始化Isaac Sim  
    simulation_app = SimulationApp({"headless": False})  
    
    # 创建测试实例  
    tester = JointTester(  
        usd_path="/path/to/your/model.usd",  
        root_prim_path="/Robot"  
    )  
    
    # 交互式测试流程  
    try:  
        # 步骤1：列出所有关节  
        tester.list_joints()  
        
        # 步骤2：选择要测试的关节  
        selected_index = int(input("请输入要测试的关节编号: ")) - 1  
        tester.select_joint(selected_index)  
        
        # 步骤3：配置驱动模式  
        drive_mode = input("选择驱动模式 (position/velocity/force): ").lower()  
        tester.configure_drive(  
            drive_mode=drive_mode,  
            max_force=1000.0,  
            damping=50.0,  
            stiffness=100.0 if drive_mode == "position" else 0.0  
        )  
        
        # 步骤4：发送测试指令  
        target_value = float(input("输入目标值: "))  
        tester.send_command(target_value)  
        
        # 步骤5：启动监控  
        tester.monitor_joint(duration=5.0)  
        
    except Exception as e:  
        carb.log_error(f"测试出错: {str(e)}")  
    finally:  
        simulation_app.close()