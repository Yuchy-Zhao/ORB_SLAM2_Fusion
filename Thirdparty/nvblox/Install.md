# Redistributable Building 
This will walk you through how to build a single libnvblox shared library with only static dependencies. The dependencies must be built manually with -fPIC (position independent code) and linked as static libraries.  
You generally shouldn't need to do this except for building nvblox to be included in Bazel for example; the exported targets should work fine for inclusion in CMake projects.

## Structure
The folder structure should look like:

```bash
├── nvblox/
├── glog-0.4.0/
├── sqlite-autoconf-3390400/
├── gflags-2.2.2/
```

## Step 1: Install dependencies
We need `glog`, `sqlite3`, and `gflags` prepared for static linking. To get these ready do:
**Sqlite3**:  
```bash
wget https://sqlite.org/2022/sqlite-autoconf-3400100.tar.gz
tar -xvf sqlite-autoconf-3400100.tar.gz
cd sqlite-autoconf-3400100
mkdir install
CFLAGS=-fPIC ./configure --prefix=$PWD/install/
make
make install
cd ..
```

**glog**:  
```bash
wget https://github.com/google/glog/archive/refs/tags/v0.4.0.tar.gz
tar -xvf v0.4.0.tar.gz
cd glog-0.4.0
mkdir build
mkdir install
cd build
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=$PWD/../install/ -DWITH_GFLAGS=OFF -DBUILD_SHARED_LIBS=OFF
make -j8
make install
cd ../..
```

**gflags**:  
```bash
wget https://github.com/gflags/gflags/archive/refs/tags/v2.2.2.tar.gz
tar -xvf v2.2.2.tar.gz
cd gflags-2.2.2/
mkdir build
mkdir install
cd build
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=$PWD/../install/ -DGFLAGS_BUILD_STATIC_LIBS=ON -DGFLAGS=google && make -j8 && make install
cd ../..
```

## Step 2: Build nvblox
Build nvblox with the paths set to where you've unzipped and built the dependencies:
```
cd nvblox/
mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/home/yuchy/Dev/nvblox/nvblox/install/ -DBUILD_FOR_ALL_ARCHS=TRUE -DBUILD_REDISTRIBUTABLE=TRUE -DSQLITE3_BASE_PATH="/home/yuchy/Dev/nvblox/sqlite-autoconf-3400100/install/" -DGLOG_BASE_PATH="/home/yuchy/Dev/nvblox/glog-0.4.0/install/" -DGFLAGS_BASE_PATH="/home/yuchy/Dev/nvblox/gflags-2.2.2/install/"
make -j8 && make install
```