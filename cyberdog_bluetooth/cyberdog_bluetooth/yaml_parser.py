#!/usr/bin/python3
#
# Copyright (c) 2022 Xiaomi Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import yaml


class YamlParser:

    @staticmethod
    def GetYamlData(yaml_file):
        data = None
        try:
            file = open(yaml_file, 'r', encoding='utf-8')
            print('reading yaml file')
            file_data = file.read()
            file.close()
            data = yaml.load(file_data, yaml.FullLoader)
        except FileNotFoundError:
            print('yaml file not found')
        return data

    @staticmethod
    def GenerateYamlDoc(obj, yaml_file):
        result = False
        try:
            file = open(yaml_file, 'w', encoding='utf-8')
            yaml.dump(obj, file)
            file.close()
            result = True
        except FileNotFoundError:
            print('yaml file with wrong path')
        return result
