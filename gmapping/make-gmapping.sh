#install dependencies
#sudo apt-get install qt3-dev-tools

mkdir gmapping
cd gmapping

#get and unpack source
wget http://pr.willowgarage.com/downloads/gmapping_r39.tar.gz
tar -xvf gmapping_r39.tar.gz
mv gmapping_export/* .
rmdir gmapping_export

#apply patch
patch -p0 < ../gmapping-r39.patch

#configure
./configure

#make
make
