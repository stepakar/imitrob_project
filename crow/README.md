# crow

Collaborative Robotic Workplace of the Future





### Running Speech Recognition Nodes

1) install requirements: <p>
`pip3 install pyyaml rospkg pyaudio google-cloud-speech`
2) in one terminal/PyCharm, export path to your google cloud account credentials (or ask Gabina for her file - temporary solution): <p>
`export GOOGLE_APPLICATION_CREDENTIALS="crow/data/MyFirstProject.json"`
3) run node for continuous recognition (/asr_node): <p>
`python3 asr/nodes/CloudStream.py`
4) in second terminal, run nl_input_node from [crow_nlp](https://gitlab.ciirc.cvut.cz/imitrob/crow_nlp): <p>
`python3 /crow_nlp/scripts/nl_input_node_asr.py`
