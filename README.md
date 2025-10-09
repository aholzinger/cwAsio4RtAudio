# cwAsio4RtAudio
cwASIO demo driver that uses RtAudio as a backend

cwAsio4RtAudio is similar to ASIO4ALL with the difference that it runs on Linux (currently only tested (compiled/linked/ran on Linux) and that instead of Steinberg's ASIO[^1] code it uses s13n's cwASIO: https://github.com/s13n/cwASIO

[^1]: ASIO is a trademark of Steinberg Media Technologies GmbH.

With cwAsio4RtAudio a developer who wants to make his ASIO application running also on Linux can use cwASIO instead of Steinberg's ASIO SDK. To verify if his/her efforts were successful he/she can use the cwAsio4RtAudio driver to test against.

To allow cwAsio4RtAudio being found as a cwASIO driver (for the moment) you have to do the following:
1. create the directory /etc/cwASIO
2. create the directory /etc/cwASIO/cwAsio4RtAudio
3. create a file named "driver" (without the quotes) in /etc/cwASIO/cwAsio4RtAudio
4. this file needs to hold just one line with the full absolute path to the cwAsio4RtAudioDriver.so (the build result of this repository)
5. if you want you can create also a file "description" (without the quotes) in the very same directory with a short single line description like for example "cwAsio4RtAudio: cwASIO for RtAudio driver"

You can define which audio devices and samplerates, etc. to use by copying the file "config" to either /etc/cwASIO/cwAsio4RtAudio (system wide settings) or ~/.cwASIO/cwAsio4RtAudio. If the config file in ~/.cwASIO/cwAsio4RtAudio is found the system wide will not be used.

The driver supports multiple instances: just add another directory in /etc/cwASIO with a different name, but let the "driver" file point to the same DLL/shared object. The same applies for the "config" file stored in the driver's subdirectory inside the ~/.cwASIO directory.
