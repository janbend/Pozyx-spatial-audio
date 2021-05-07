# processing configuration

## prerequisite
- download processing ide (tested V3.54)
- include controlP5, oscP5, netP5 libary with processing libary manager
- include gwoptics.jar libary manually by copying into processing core folder
- (more details in dependencies.txt file)

## required code modifications
- define com port of your pozyx workstation tag (prcoessing = arduino port), e.g COM8
- define local host ip-address (e.g. 127.0.0.1)
- define osc port for osc interface of digital audio workstation
- define room dimension (room_x, room_y) - distance between anchor coordinates in meter

## setup
- verify the code and check for missing libaries or other issues in processing ide
- successful patch, if pozyx remote tag and anchors are visualized and sensor data is presented
- otherwise, test the [official pozyx orientation3D](https://github.com/pozyxLabs/Pozyx-processing)

## digital audio workstation
- download and install reaper
- open a new project and navigate to options -> preferences -> control/OSC/web
- add new control surface -> select control surface mode OSC (Open Soujnd Control)
- define device name and select Device IP/port
- define osc port e.g. 8888 and device ip-address e.g. 127.0.0.1
- allow binding messages to REAPER actions and FX learn
- (guidance on how to setup an spatial audio scene in reaper can be requested)

## vst plugin
- add vst to effects (FX) of selected channel (e.g. master bus)
- navigate to MIDI/OSC learn in parameter list (Param -> FX paramater list -> Learn -> Select paramater)
- to bind specific OSC message to MIDI paramater you have to deactivate all others in processing gui (otherwise overflow)
- (guidance on how to setup an spatial audio scene in reaper can be requested)

## osc commands
```math
\begin{table}[h!]
\centering
\begin{center}
\newcolumntype{M}[1]{>{\centering\arraybackslash}m{#1}}
\begin{tabular}{ |p{4cm}|M{3cm}|M{6cm}|}
\hline
 \multicolumn{3}{|c|}{OSC-Nachrichten des Medienservers} \\
 \hline
  \textbf{Parameter} & \textbf{Index} & \textbf{Audio Plug-ins} \\
 \hline
 Azimut-Winkel & /imu\_az/ & DearVR: Azimuth \\
 \hline
 Elevation-Winkel & /imu\_el/ & DearVR: Elevation \\
\hline
 Yaw-Winkel & /imu\_x/ &  AmbiX, IEM-Suite, Sparta \\
\hline
 Yaw-Winkel (invertiert) & /imu\_x\_inv/ &  AmbiX, IEM-Suite, Sparta \\
\hline
 Roll-Winkel & /imu\_y/ &  AmbiX, IEM-Suite, Sparta \\
\hline
 Pitch-Winkel & /imu\_z/ &  AmbiX, IEM-Suite, Sparta \\
\hline
 Position X & /pos\_x/ &  IEM-Suite: X-Koordinate \\
\hline
 Position Y & /pos\_y/ &  IEM-Suite: Y-Koordinate \\
\hline
 Position Z & /pos\_z/ &  IEM-Suite: Z-Koordinate \\
\hline
 Position Y & /dearx/ &  DearVR: X-Koordinate \\
\hline
 Position X & /dearz/ &  DearVR: Z-Koordinate \\
\hline
 Zone 1 & /zone1/ &  Reaper: Zone 1 Lautstärke \\
\hline
 Zone 1 & /marker\_zone1/ &  Reaper: Zone 1 Markierung \\
\hline
 Zone 2 & /zone2/ &  Reaper: Zone 2 Lautstärke \\
\hline
 Zone 2 & /marker\_zone2/ &  Reaper: Zone 2 Markierung \\
\hline
 Zone 3 & /zone3/ &  Reaper: Zone 3 Lautstärke \\
\hline
 Zone 3 & /marker\_zone3/ &  Reaper: Zone 3 Markierung \\
\hline
 Zone 4 & /zone4/ &  Reaper: Zone 4 Lautstärke \\
\hline
 Zone 4 & /marker\_zone4/ &  Reaper: Zone 4 Markierung \\
\hline
\end{tabular}
\end{center}
\caption{OSC-Nachrichten des Medienservers}
\label{table:1}
\end{table}
```
