#!/bin/zsh

./bin/paper_experiments  >> experiments.txt
bash toggle_formulation.bash ./bin/config.json
./bin/paper_experiments  >> experiments.txt
bash toggle_init.bash ./bin/config.json
./bin/paper_experiments  >> experiments.txt
bash toggle_formulation.bash ./bin/config.json
./bin/paper_experiments  >> experiments.txt
bash toggle_init.bash ./bin/config.json

# increment the init_rank value and run the experiments
bash increment_init_rank.bash ./bin/config.json
./bin/paper_experiments >> experiments.txt
bash toggle_formulation.bash ./bin/config.json
./bin/paper_experiments  >> experiments.txt
bash toggle_init.bash ./bin/config.json
./bin/paper_experiments  >> experiments.txt
bash toggle_formulation.bash ./bin/config.json
./bin/paper_experiments  >> experiments.txt
bash toggle_init.bash ./bin/config.json
