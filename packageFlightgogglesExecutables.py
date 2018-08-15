#!/usr/bin/env python2

import sys, subprocess, os, shutil
import os.path as path
import fire

# Usage: ./packageFlightgogglesExecutables.sh /media/medusa/BCAE9765AE9716CC/FlightGogglesStaging/v1.5.6/

def packageFlightgogglesExecutables(base_dir="/media/medusa/BCAE9765AE9716CC/FlightGogglesStaging/", version="v1.5.6"):
    # Find directory for the executable version
    sourceDir = path.expanduser(path.join(base_dir, version))

    platformFolders = ["Linux64", "macOS", "Windows64"]
    
    # Unity builder uses different conventions for the platform folder and the platform name in the linux executable
    platformExecutableLookup = { "Linux64": "Linux_64"}


    for platform in platformFolders:
        # Get platform string to use in tarball
        platformStringForTarball = platformExecutableLookup.get(platform, platform)

        print "Packing platform: %s" % platform

        platformFolder = path.join(sourceDir, platform)
        tarballName = "FlightGoggles_%s_%s.tar.gz" % (version, platformStringForTarball)
        tarballPath = path.join(platformFolder, tarballName)

        # Run pigz compressor
        command = "tar -c --use-compress-program=pigz -f %s *" % tarballPath
        process = subprocess.Popen(command, shell=True, cwd=platformFolder)
        process.wait()
        print "Finished compressing %s" % tarballName

        # Move tarball to the base_dir
        shutil.move(tarballPath, sourceDir)




####################
# MAIN FUNCTION
####################

if __name__ == '__main__':
    fire.Fire(packageFlightgogglesExecutables)
