@if "%DEBUG%" == "" @echo off
@rem ##########################################################################
@rem
@rem  auction-based-mrta startup script for Windows
@rem
@rem ##########################################################################

@rem Set local scope for the variables with windows NT shell
if "%OS%"=="Windows_NT" setlocal

set DIRNAME=%~dp0
if "%DIRNAME%" == "" set DIRNAME=.
set APP_BASE_NAME=%~n0
set APP_HOME=%DIRNAME%..

@rem Add default JVM options here. You can also use JAVA_OPTS and AUCTION_BASED_MRTA_OPTS to pass JVM options to this script.
set DEFAULT_JVM_OPTS=

@rem Find java.exe
if defined JAVA_HOME goto findJavaFromJavaHome

set JAVA_EXE=java.exe
%JAVA_EXE% -version >NUL 2>&1
if "%ERRORLEVEL%" == "0" goto init

echo.
echo ERROR: JAVA_HOME is not set and no 'java' command could be found in your PATH.
echo.
echo Please set the JAVA_HOME variable in your environment to match the
echo location of your Java installation.

goto fail

:findJavaFromJavaHome
set JAVA_HOME=%JAVA_HOME:"=%
set JAVA_EXE=%JAVA_HOME%/bin/java.exe

if exist "%JAVA_EXE%" goto init

echo.
echo ERROR: JAVA_HOME is set to an invalid directory: %JAVA_HOME%
echo.
echo Please set the JAVA_HOME variable in your environment to match the
echo location of your Java installation.

goto fail

:init
@rem Get command-line arguments, handling Windows variants

if not "%OS%" == "Windows_NT" goto win9xME_args

:win9xME_args
@rem Slurp the command line arguments.
set CMD_LINE_ARGS=
set _SKIP=2

:win9xME_args_slurp
if "x%~1" == "x" goto execute

set CMD_LINE_ARGS=%*

:execute
@rem Setup the command line

set CLASSPATH=%APP_HOME%\lib\auction-based-mrta.jar;%APP_HOME%\lib\coordination_oru-0.5.4.jar;%APP_HOME%\lib\meta-csp-framework-master-SNAPSHOT.jar;%APP_HOME%\lib\aima-core-0.11.0.jar;%APP_HOME%\lib\jna-4.4.0.jar;%APP_HOME%\lib\jgrapht-core-1.0.1.jar;%APP_HOME%\lib\reflections-0.9.11.jar;%APP_HOME%\lib\rosjava-0.3.6.jar;%APP_HOME%\lib\visualization_msgs-1.12.7.jar;%APP_HOME%\lib\nav_msgs-1.12.7.jar;%APP_HOME%\lib\tf2_msgs-0.5.20.jar;%APP_HOME%\lib\geometry_msgs-1.12.7.jar;%APP_HOME%\lib\rosjava_test_msgs-0.3.0.jar;%APP_HOME%\lib\rosgraph_msgs-1.11.2.jar;%APP_HOME%\lib\actionlib_msgs-1.12.7.jar;%APP_HOME%\lib\std_msgs-0.5.11.jar;%APP_HOME%\lib\message_generation-0.3.3.jar;%APP_HOME%\lib\websocket-server-9.4.12.v20180830.jar;%APP_HOME%\lib\websocket-client-9.4.12.v20180830.jar;%APP_HOME%\lib\websocket-common-9.4.12.v20180830.jar;%APP_HOME%\lib\websocket-servlet-9.4.12.v20180830.jar;%APP_HOME%\lib\websocket-api-9.4.12.v20180830.jar;%APP_HOME%\lib\gson-2.8.5.jar;%APP_HOME%\lib\org.ow2.sat4j.sat-2.3.4.jar;%APP_HOME%\lib\jung-graph-impl-2.0.1.jar;%APP_HOME%\lib\jung-visualization-2.0.1.jar;%APP_HOME%\lib\jung-algorithms-2.0.1.jar;%APP_HOME%\lib\prefuse-beta-20071021.jar;%APP_HOME%\lib\jfreechart-1.0.13.jar;%APP_HOME%\lib\dnsjava-2.1.1.jar;%APP_HOME%\lib\apache_xmlrpc_server-0.3.6.jar;%APP_HOME%\lib\apache_xmlrpc_client-0.3.6.jar;%APP_HOME%\lib\apache_xmlrpc_common-0.3.6.jar;%APP_HOME%\lib\ws-commons-util-1.0.1.jar;%APP_HOME%\lib\junit-4.8.1.jar;%APP_HOME%\lib\jts-1.13.jar;%APP_HOME%\lib\VectorGraphics2D-0.12.jar;%APP_HOME%\lib\universal-tween-engine-6.3.3.jar;%APP_HOME%\lib\guava-20.0.jar;%APP_HOME%\lib\javassist-3.21.0-GA.jar;%APP_HOME%\lib\netty-3.5.2.Final.jar;%APP_HOME%\lib\com.springsource.org.apache.commons.httpclient-3.1.0.jar;%APP_HOME%\lib\com.springsource.org.apache.commons.codec-1.3.0.jar;%APP_HOME%\lib\com.springsource.org.apache.commons.io-1.4.0.jar;%APP_HOME%\lib\commons-pool-1.6.jar;%APP_HOME%\lib\com.springsource.org.apache.commons.lang-2.4.0.jar;%APP_HOME%\lib\gradle_plugins-0.3.3.jar;%APP_HOME%\lib\com.springsource.org.apache.commons.logging-1.1.1.jar;%APP_HOME%\lib\com.springsource.org.apache.commons.net-2.0.0.jar;%APP_HOME%\lib\jetty-servlet-9.4.12.v20180830.jar;%APP_HOME%\lib\jetty-client-9.4.12.v20180830.jar;%APP_HOME%\lib\jetty-security-9.4.12.v20180830.jar;%APP_HOME%\lib\jetty-server-9.4.12.v20180830.jar;%APP_HOME%\lib\jetty-http-9.4.12.v20180830.jar;%APP_HOME%\lib\jetty-xml-9.4.12.v20180830.jar;%APP_HOME%\lib\jetty-io-9.4.12.v20180830.jar;%APP_HOME%\lib\jetty-util-9.4.12.v20180830.jar;%APP_HOME%\lib\org.ow2.sat4j.maxsat-2.3.4.jar;%APP_HOME%\lib\org.ow2.sat4j.pb-2.3.4.jar;%APP_HOME%\lib\org.ow2.sat4j.core-2.3.4.jar;%APP_HOME%\lib\org.ow2.sat4j.core-2.3.4-tests.jar;%APP_HOME%\lib\commons-beanutils-1.6.jar;%APP_HOME%\lib\commons-cli-1.1.jar;%APP_HOME%\lib\jung-api-2.0.1.jar;%APP_HOME%\lib\collections-generic-4.01.jar;%APP_HOME%\lib\colt-1.2.0.jar;%APP_HOME%\lib\jcommon-1.0.16.jar;%APP_HOME%\lib\javax.servlet-api-3.1.0.jar;%APP_HOME%\lib\commons-logging-1.0.jar;%APP_HOME%\lib\commons-collections-2.0.jar;%APP_HOME%\lib\concurrent-1.3.4.jar;%APP_HOME%\lib\xml-apis-1.0.b2.jar

@rem Execute auction-based-mrta
"%JAVA_EXE%" %DEFAULT_JVM_OPTS% %JAVA_OPTS% %AUCTION_BASED_MRTA_OPTS%  -classpath "%CLASSPATH%" oru.smarter.TestReusableResourceScheduler %CMD_LINE_ARGS%

:end
@rem End local scope for the variables with windows NT shell
if "%ERRORLEVEL%"=="0" goto mainEnd

:fail
rem Set variable AUCTION_BASED_MRTA_EXIT_CONSOLE if you need the _script_ return code instead of
rem the _cmd.exe /c_ return code!
if  not "" == "%AUCTION_BASED_MRTA_EXIT_CONSOLE%" exit 1
exit /b 1

:mainEnd
if "%OS%"=="Windows_NT" endlocal

:omega
