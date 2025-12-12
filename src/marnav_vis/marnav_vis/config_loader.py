"""
通用YAML配置加载工具
用于从YAML文件加载ROS2节点参数
"""
import yaml
import os
from pathlib import Path


class ConfigLoader:
    """YAML配置文件加载器"""
    
    def __init__(self, config_file_path: str):
        """
        初始化配置加载器
        
        Args:
            config_file_path: YAML配置文件路径
        """
        self.config_file_path = config_file_path
        self.config = None
        
        if not os.path.exists(config_file_path):
            raise FileNotFoundError(f"配置文件不存在: {config_file_path}")
        
        self._load_config()
    
    def _load_config(self):
        """从YAML文件加载配置"""
        try:
            with open(self.config_file_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)
            print(f"✅ 成功加载配置文件: {self.config_file_path}")
        except Exception as e:
            raise RuntimeError(f"加载配置文件失败: {e}")
    
    def get(self, *keys, default=None):
        """
        获取配置项（支持多层级访问）
        
        Args:
            *keys: 配置路径，例如 get('camera', 'width_height')
            default: 默认值
            
        Returns:
            配置值或默认值
        """
        result = self.config
        try:
            for key in keys:
                result = result[key]
            return result
        except (KeyError, TypeError):
            return default
    
    def get_camera_config(self):
        """获取相机配置"""
        return self.get('camera', default={})
    
    def get_gnss_config(self):
        """获取GNSS配置"""
        return self.get('gnss', default={})
    
    def get_ais_config(self):
        """获取AIS配置"""
        return self.get('ais', default={})
    
    def get_deepsorvf_config(self):
        """获取DeepSORVF配置"""
        return self.get('DeepSORVF', default={})
    
    @staticmethod
    def find_config_file(package_name: str, relative_path: str = None) -> str:
        """
        查找配置文件路径
        
        Args:
            package_name: ROS2包名
            relative_path: 相对于包的config目录的路径
            
        Returns:
            配置文件绝对路径
        """
        try:
            from ament_index_python.packages import get_package_share_directory
            package_path = get_package_share_directory(package_name)
            if relative_path:
                return os.path.join(package_path, 'config', relative_path)
            else:
                return os.path.join(package_path, 'config')
        except Exception:
            # 如果ament_index不可用，尝试使用相对路径
            current_dir = Path(__file__).parent.parent
            if relative_path:
                return str(current_dir / 'config' / relative_path)
            else:
                return str(current_dir / 'config')

