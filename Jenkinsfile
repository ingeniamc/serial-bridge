properties([
    buildDiscarder(logRotator(artifactNumToKeepStr: '1', daysToKeepStr: '1')),
])

node('xcore') {

    def version_pre = '1.0.0'
	
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
        sh '/opt/Atollic_TrueSTUDIO_for_STM32_x86_64_9.3.0/ide/TrueSTUDIO --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data coco_workspace -importAll examples/STM32F407_Discovery/build/TrueSTUDIO'
        sh '/opt/Atollic_TrueSTUDIO_for_STM32_x86_64_9.3.0/ide/TrueSTUDIO --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data coco_workspace -importAll examples/STM32F042_Bridge/LiteCoco/TrueSTUDIO'
    }

    stage ('Build projects') {
        sh '/opt/Atollic_TrueSTUDIO_for_STM32_x86_64_9.3.0/ide/TrueSTUDIO --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data coco_workspace -cleanBuild ccore-lite/Release'
        sh '/opt/Atollic_TrueSTUDIO_for_STM32_x86_64_9.3.0/ide/TrueSTUDIO --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data coco_workspace -cleanBuild LiteCoco/Release'
    }
    
    stage ('Archive outputs') {
        archiveArtifacts artifacts: '**/*.hex'
    }
    
    if (env.BRANCH_NAME != 'develop') {    
        stage('Publish linux outputs') {
            if (env.BRANCH_NAME == 'master'){
                sh """ 
                    mkdir -p /mnt/dist/turonet/${version_pre}
                    
                    cp LiteCoco.hex /mnt/dist/turonet/${version_pre}/turonet_${version_pre}.hex
                   """
            }
            else {
                sh """ 
                    mkdir -p /mnt/dist/turonet/candidate_${version_pre}
                    
                    cp LiteCoco.hex /mnt/dist/turonet/candidate_${version_pre}/turonet_${version_pre}.hex
                   """
            }
        }
    }
}
