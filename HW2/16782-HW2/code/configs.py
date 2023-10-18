import random
import math

def generate_joint_angles(dof, num_configs=20):
    all_configs = []
    
    for _ in range(num_configs):
        config = [random.uniform(0, 2*math.pi) for _ in range(dof)]
        all_configs.append(config)
        
    return all_configs

dof = 6
random_configs = generate_joint_angles(dof)

# display configs
for i, config in enumerate(random_configs):
    print(f"Configuration {i+1}: {config}")
