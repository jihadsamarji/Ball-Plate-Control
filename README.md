-------------------------------------
The Project is Still under Construction
-------------------------------------
# Ball-Plate-Control
This is a project done for a Master Thesis in the Electromechanical Engineering (specialization: Mechatronics),
it is about the design and control of a Ball and Plate Didactic Device.
This device is intended to be developed into a Commercial device and maybe sold to educational institutes or young apprentices in the field of Control Systems. 

The Project is Explained entirely on : [Presentation Video](https://youtu.be/tS8yI4Vpe_U) 

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

things you need to install before running the code:
- Arduino IDE
- Python 3.7
- Python PiP
- Python IDE (Pycharm is recommended)

Run the following commands in your command terminal after installing "PIP"
```
pip install opencv-python==3.4.5.20
pip install Django
pip install Pillow
pip install image
pip install imutils
pip install numpy
pip install pyserial
pip install pytz
pip install setuptools
pip install sqlparse
pip install wheel
```

## Deployment

- Make sure you run the "solveequation.py" code first to generate the dictionary file. (found in /Software/Lookuptable/)
- Then place the dictionary file called "data.txt" in the same directory of the "interface.py" code. (found in /Software/Interface/)
- Connect an Arduino and upload the arduino code "Full_Program.ino" using the arduino IDE.
- Make sure you have a Camera attached on your system. (otherwise, you will get an opencv error when running)
- Run the code "interface.py" . (found in /Software/Interface/) 

## Simulation Files
The simulation files are found in ( /Software/Matlab & Simulation/)

## Contributing

Please Contact me on jmz_mmz@hotmail.com for any contribution requests.

## Versioning

Versioning was done using GIT

## Authors

* **Jihad Alsamarji** - *Masters in Electromechanical Engineering* - [JISA_leb](https://github.com/jihadsamarji)

See also the list of [contributors](https://github.com/jihadsamarji/Ball-Plate-Control/graphs/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

I would first like to thank my thesis advisors and supervisors of the SAAS department at Université Libre de Bruxelles; Micheal KINNAERT, Laurent CATOIRE,Christoph MERTENS.
The door to my supervisors was always open whenever I ran into problems or had a question about my research or writing.
They consistently allowed this paper to be my own work, but steered me in the right direction whenever I needed it.
I would also like to thank the members of my dissertation committee - not only for their time and extreme patience, but for their intellectual contributions to my development as an Engineer.
In Addition, I like to express my gratitude to the open source Community that helped a lot in overcoming software and computational challenges during the study.
I must express my very profound gratitude to my parents, friends and to my girlfriend for providing me with unfailing support and continuous encouragement throughout my study and through the process of researching and writing this thesis.
This accomplishment would not have been possible without them.
Thank you.
