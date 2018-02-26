properties([
    buildDiscarder(logRotator(artifactNumToKeepStr: '10', daysToKeepStr: '30')),
])

node('xcore') {
	
	deleteDir()
    
    stage('Checkout') {
        checkout scm
        sh 'git submodule update --init --recursive'
    }
    
    stage('Importing projects') {
        sh  '''
            if [ ! -d coco_workspace ]; then
                mkdir coco_workspace
            fi
            '''
        sh  '/opt/Atollic_TrueSTUDIO_for_STM32_x86_64_9.0.0/ide/TrueSTUDIO --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data coco_workspace -importAll build/TrueSTUDIO'
    }

    stage ('Build projects') {
        sh  '/opt/Atollic_TrueSTUDIO_for_STM32_x86_64_9.0.0/ide/TrueSTUDIO --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data coco_workspace -cleanBuild ccore-lite/Release'
    }
}
