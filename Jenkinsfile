pipeline {
  agent {
    dockerfile {
      filename 'docker/Dockerfile'
      additionalBuildArgs '--pull'
      reuseNode true
    }

  }
  stages {
    stage('gcc debug') {
      steps {
        sh '''
             if [ ! -d "build" ]; then
                mkdir build;
             fi
             cd build
             rm -rf *
             cmake -DCMAKE_BUILD_TYPE=Debug ..
             make all
             make install
             ctest
           '''
      }
    }
    stage('gcc release') {
      steps {
        sh '''
             if [ ! -d "build" ]; then
                mkdir build;
             fi
             cd build
             rm -rf *
             cmake -DCMAKE_BUILD_TYPE=Release ..
             make all
             make install
             ctest
           '''
      }
    }
    stage('gcc native') {
      steps {
        sh '''
             if [ ! -d "build" ]; then
                mkdir build;
             fi
             cd build
             rm -rf *
             cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_NATIVE=1 ..
             make all
             make install
             ctest
           '''
      }
    }
  }
}
