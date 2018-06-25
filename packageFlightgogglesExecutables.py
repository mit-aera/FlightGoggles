#!/usr/bin/env python2

import sys, subprocess, os, shutil
import os.path as path
import fire

# Usage: ./packageFlightgogglesExecutables.sh /media/medusa/BCAE9765AE9716CC/FlightGogglesStaging/ v1.5.4-rc1
# Will package EXEs that have the form FlightGogglesStaging/v1.5.4-rc1/<platform>

def packageFlightgogglesExecutables(base_dir, version):
    platformFolders = ["Linux64", "macOS", "Windows64"]
    
    # Unity builder uses different conventions for the platform folder and the platform name in the linux executable
    platformExecutableLookup = { "Linux64": "Linux_64"}


    for platform in platformFolders:
        # Get platform string to use in tarball
        platformStringForTarball = platformExecutableLookup.get(platform, platform)
        platformFolder = path.join(base_dir, version, platform)

        print "" # newline

        print "Packing platform %s at folder %s" % (platform, platformFolder)


        # Check that this folder exists
        if not path.exists(platformFolder):
            print "WARNING: Platform folder %s does not exist. Skipping." % platformFolder
            continue

        tarballName = "FlightGoggles_%s_%s.tar.gz" % (version, platformStringForTarball)
        tarballPath = path.join(platformFolder, tarballName)

        # Run pigz compressor
        command = "tar -c --use-compress-program=pigz -f %s *" % tarballPath
        process = subprocess.Popen(command, shell=True, cwd=platformFolder)
        process.wait()
        print "Finished compressing %s" % tarballName

        # Move tarball to the base_dir
        destinationTarballName = path.join(base_dir, tarballName)
        try:
            os.remove(destinationTarballName)
        except OSError:
            pass
        shutil.move(tarballPath, base_dir)




####################
# MAIN FUNCTION
####################

if __name__ == '__main__':
    fire.Fire(packageFlightgogglesExecutables)
