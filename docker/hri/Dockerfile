# Use the official image as a parent image.
FROM rasberry:latest
WORKDIR /staging
USER root
#cogutil dependencies -- not found: cxxtest
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | apt-key add -
RUN echo "deb https://apt.kitware.com/ubuntu/ bionic main" | tee '/etc/apt/sources.list.d/cmake.list'
RUN apt update -y
RUN apt install -y guile-2.2-dev build-essential cmake rlwrap libiberty-dev libicu-dev libbz2-dev python3-dev python3-setuptools python3-nose libboost-date-time-dev libboost-filesystem-dev libboost-math-dev libboost-program-options-dev libboost-regex-dev libboost-serialization-dev libboost-thread-dev libboost-system-dev libboost-random-dev libzmq3-dev libtbb-dev binutils-dev libpq-dev uuid-dev libprotoc-dev protobuf-compiler libsdl-gfx1.2-dev libssl-dev tcl-dev tcsh libfreetype6-dev libatlas-base-dev gfortran gearman libgearman-dev ccache libgsasl7 libldap2-dev krb5-multidev wordnet wordnet-dev wordnet-sense-index libatomic-ops-dev libgmp-dev libffi-dev libreadline-dev libcpprest libltdl-dev odbc-postgresql postgresql-client netcat-openbsd liboctomap-dev locales unixodbc python-igraph

WORKDIR /staging
#RUN git clone https://github.com/opencog/cogutil.git
#WORKDIR /staging/cogutil
#RUN git checkout f14ab80cbf02a78677c619bc1fc1b5bbcfedc6f1
WORKDIR /staging/cogutil/build
#RUN cmake -DCMAKE_DISABLE_FIND_PACKAGE_Python3Interp=TRUE ..
#RUN make install -j$(nproc)

WORKDIR /staging
#RUN git clone https://github.com/opencog/atomspace.git
#WORKDIR /staging/atomspace
#RUN git checkout eae014544307b4774dd48bddebb195cfd54075e6
WORKDIR /staging/atomspace/build
#RUN cmake -DCMAKE_DISABLE_FIND_PACKAGE_Python3Interp=TRUE ..
#RUN make install -j$(nproc)

WORKDIR /staging
#RUN git clone https://github.com/alexander-gabriel/ure.git
#WORKDIR /staging/ure
#RUN git checkout e899549cdd3a1d435c2eefc8aadef02fc2f8cf6d
WORKDIR /staging/ure/build
#RUN cmake -DCMAKE_DISABLE_FIND_PACKAGE_Python3Interp=TRUE ..
#RUN make install -j$(nproc)

WORKDIR /staging
#RUN git clone https://github.com/alexander-gabriel/pln.git
#WORKDIR /staging/pln
#RUN git checkout e85567236bb60cbe02a5215a2b57d8c7ccfe6b43
WORKDIR /staging/pln/build
#RUN cmake -DCMAKE_DISABLE_FIND_PACKAGE_Python3Interp=TRUE ..
#RUN make install -j$(nproc)

WORKDIR /staging
#RUN git clone https://github.com/opencog/opencog.git
#WORKDIR /staging/opencog
#RUN git checkout fc3cb0dd12dc205054584e486be2b69443494e2a
#WORKDIR /staging/opencog/build
#RUN cmake -DCMAKE_DISABLE_FIND_PACKAGE_Python3Interp=TRUE ..
#RUN make install -j$(nproc)

#WORKDIR /staging
#RUN git clone https://github.com/opencog/cogserver.git
#WORKDIR /staging/cogserver
#RUN git checkout 66d41f8a601837490e19dce870b50e75513adcf9
#WORKDIR /staging/cogserver/build
#RUN cmake -DCMAKE_DISABLE_FIND_PACKAGE_Pytho3Interp=TRUE ..
#RUN make install -j$(nproc)


USER rasberry
WORKDIR /home/rasberry

EXPOSE 22
USER root
CMD ["/usr/sbin/sshd", "-D"]
