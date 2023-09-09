#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""

import os
import requests
from typing import Optional

from ..common.config import Settings
from ..common.logger import Logger
from ..common.utils import load_yaml


class FileProcess():
    def __init__(self, settings: Settings, logger: Logger):
        self.settings = settings
        self.logger = logger

    def download_to_save(self, url: str, version: Optional[str] = None) -> bool:
        """下载"""
        dir_path = self.settings.road_path
        os.makedirs(dir_path, exist_ok=True)
        v_file_path = os.path.join(dir_path, 'road.version')

        if os.path.exists(v_file_path):
            with open(v_file_path, 'r') as f:
                _version = f.read()
                if _version and version and str(version) == str(_version):
                    return False

        try:
            response = requests.get(url=url, timeout=5, verify=False)
            filename = 'road.json'
            if 'content-disposition' in response.headers:
                content_disposition = response.headers['content-disposition']
                filename_start_index = content_disposition.find('filename=')
                if filename_start_index != -1:
                    filename = content_disposition[filename_start_index + len('filename='):]
                    filename = filename.strip("\"'")

            file_path = os.path.join(dir_path, f"{filename}.bak")

            with open(file_path, 'wb') as f:
                f.write(response.content)
            os.rename(file_path, os.path.join(dir_path, filename))

            with open(v_file_path, 'wb') as f:
                f.write(str(version).encode('utf-8'))  # type: ignore

            self.logger.sys_log.info(f"下载路网成功 {file_path} 版本{version}")
            return True
        except requests.exceptions.Timeout:
            self.logger.sys_log.error(f"下载路网超时 {url}")
        except requests.exceptions.RequestException as e:
            self.logger.sys_log.error(f"下载路网错误 {url}", e)
        except Exception as e:
            self.logger.sys_log.error(f"下载路网错误 {url}", e)
        return False

    def read_to_upload(self, url: str, version: Optional[str] = None):
        dir_path = self.settings.maps_path
        os.makedirs(dir_path, exist_ok=True)
        file_names = []
        for file_name in os.listdir(dir_path):
            file_path = os.path.join(dir_path, file_name)
            if os.path.isfile(file_path):
                if file_path.endswith('yaml'):
                    metadata = load_yaml(file_path)
                    _version = metadata['version'] if 'version' in metadata else None
                    if _version and version and str(version) == str(_version):
                        self.logger.sys_log.info(f"版本一致, 放弃上传地图 {version}")
                        return
                file_names.append(file_name)

        files = []
        for file_name in file_names:
            files.append(
                ('files', (file_name, open(os.path.join(dir_path, file_name), 'rb'), 'application/octet-stream')))

        if len(files) > 0:
            # 查看是不是 yaml，读取版本进行处理   
            try:
                response = requests.post(url=url, files=files, timeout=5, verify=False)  # type: ignore
                if response.status_code == 200:
                    self.logger.sys_log.info(f"上传地图文件成功 {file_names} {version}")
                else:
                    self.logger.sys_log.error(f"上传地图文件失败 {file_names}")
            except requests.exceptions.Timeout:
                self.logger.sys_log.error(f"上传地图超时 {file_names}")
            except requests.exceptions.RequestException as e:
                self.logger.sys_log.error(f"上传地图错误 {file_names}", e)
            except Exception as e:
                self.logger.sys_log.error(f"上传地图错误 {file_names}", e)
        else:
            self.logger.sys_log.warn(f"本地无地图文件")
