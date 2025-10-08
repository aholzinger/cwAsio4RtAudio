# cwAsio4RtAudio
cwASIO demo driver that uses RtAudio as a backend

cwAsio4RtAudio is similar to ASIO4ALL with the difference that it runs on Linux (and currently only on Linux) and that instead of Steinberg's ASIO[^1] code it uses s13n's cwASIO: https://github.com/s13n/cwASIO

[^1]: ASIO is a trademark of Steinberg Media Technologies GmbH.

With cwAsio4RtAudio a developer who wants to make his ASIO application running also on Linux can use cwASIO instead of Steinberg's ASIO SDK. To verify if his/her efforts were successful he/she can use the cwAsio4RtAudio driver to test against.
