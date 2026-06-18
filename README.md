# UAV-UGV Cooperative Routing

Implementation of the Ra-DRL framework from [*An Attention-Aware Deep Reinforcement Learning Framework for UAV-UGV Collaborative Route Planning*](https://ieeexplore.ieee.org/document/10801704) (IROS 2024).

Jointly optimizes UAV–UGV routes and recharging rendezvous points to minimize mission time under energy constraints, using a transformer-based policy trained with REINFORCE.

---

## Repository Structure

```
.
├── run.py                          # Entry point — loads config and starts training
├── options.py                      # Hyperparameters (architecture, training, problem)
├── train_modified.py               # Training loop (REINFORCE)
├── eval_modified.py                # Evaluation (greedy + sampling decoding)
├── Custom_environment.py           # UAV-UGV environment with fuel dynamics
├── nets/
│   └── attention_model_modified_v3.py  # Transformer encoder-decoder policy
├── generated_scenarios/            # Reproducible test scenarios
├── results/                        # Trained model checkpoints
├── Tours_output/                   # Route outputs and heuristic baselines
```


---

## Usage

### Training

Edit hyperparameters in `options.py`, then run:

```bash
python run.py
```

### Evaluation

```bash
python eval_modified.py          # Greedy + sampling (k=256 to 10240)
python eval_modified_greedy.py   # Greedy only
```

Outputs are saved to `Tours_output/`:

- `actions_RL_greedy.csv` — greedy policy actions
- `actions_RL_sampling.csv` — sampling policy actions
- `actions_Heu_*.csv` — heuristic baseline actions (GLS, Tabu, SA)
- `scenarios.csv` — problem instances

---

## Method

- **Algorithm**: REINFORCE with greedy rollout baseline
- **Constraint handling**: Lagrangian penalty for fuel/time limits
- **Architecture**: Transformer encoder-decoder with attention
- **Decoding**: Greedy and sampling (k = 256–10240)

---

## Citation

```bibtex
@inproceedings{mondal2024attention,
  title={An attention-aware deep reinforcement learning framework for uav-ugv collaborative route planning},
  author={Mondal, Md Safwan and Ramasamy, Subramanian and Humann, James D and Dotterweich, James M and Reddinger, Jean-Paul F and Childers, Marshal A and Bhounsule, Pranav},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={13687--13694},
  year={2024},
  organization={IEEE}
}
```