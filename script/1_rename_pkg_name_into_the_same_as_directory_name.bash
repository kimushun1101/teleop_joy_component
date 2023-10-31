#!/bin/bash

# Copyright 2023 Shunsuke Kimura

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

pkg_directory=$(cd $(dirname $0)/..;pwd)
pkg_directory_name=$(basename $pkg_directory)
orig_name=$(grep -o '<name>[^<]*' $pkg_directory/package.xml | sed 's/<name>//')

if [ "$pkg_directory_name" = "$orig_name" ]; then
  echo "The current package name is the same as the directory name"
  exit
else
  find $pkg_directory -type f -not -path $pkg_directory'/.git/*' -exec sed -i "s/$orig_name/$pkg_directory_name/g" {} +
  mv $pkg_directory/include/$orig_name $pkg_directory/include/$pkg_directory_name
  mv $pkg_directory/include/$pkg_directory_name/$orig_name.hpp $pkg_directory/include/$pkg_directory_name/$pkg_directory_name.hpp
  mv $pkg_directory/src/$orig_name.cpp $pkg_directory/src/$pkg_directory_name.cpp
  echo "Rename the package name into $pkg_directory_name"
fi

