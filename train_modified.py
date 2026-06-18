import os
import time
from tqdm import tqdm
import torch
import math
import csv
from torch.utils.data import DataLoader
from torch.nn import DataParallel
from termcolor import colored, cprint
from nets.attention_model_modified_v3 import set_decode_type
from utils.log_utils import log_values
from utils import move_to


def get_inner_model(model):
    if isinstance(model, DataParallel):
        return model.module
    else:
        return model


def validate(model, dataset, opts):
    # Validate
    print('Validating...')
    cost = rollout(model, dataset, opts)
    avg_cost = cost.mean()
    print('Validation overall avg_cost: {} +- {}'.format(
        avg_cost, torch.std(cost) / math.sqrt(len(cost))))

    return avg_cost


def rollout(model, dataset, opts):
    # Put in greedy evaluation mode!
    set_decode_type(model, "greedy")
    model.eval()

    def eval_model_bat(bat):
        with torch.no_grad():
           
            cost, _ = model(move_to(bat, opts.device))
            
        return cost.data.cpu()
    
    total_samples = len(dataset)
    batches = math.ceil(total_samples / opts.eval_batch_size)
    print(f'There are {batches} batches.')
    
    return torch.cat([
        eval_model_bat(bat)
        for bat
        in tqdm(DataLoader(dataset, batch_size=opts.eval_batch_size, num_workers=0), disable=opts.no_progress_bar)
    ], 0)


def clip_grad_norms(param_groups, max_norm=math.inf):
    """
    Clips the norms for all param groups to max_norm and returns gradient norms before clipping
    :param optimizer:
    :param max_norm:
    :param gradient_norms_log:
    :return: grad_norms, clipped_grad_norms: list with (clipped) gradient norms per group
    """
    grad_norms = [
        torch.nn.utils.clip_grad_norm_(
            group['params'],
            max_norm if max_norm > 0 else math.inf,  # Inf so no clipping but still call to calc
            norm_type=2
        )
        for group in param_groups
    ]
    grad_norms_clipped = [min(g_norm, max_norm) for g_norm in grad_norms] if max_norm > 0 else grad_norms
    return grad_norms, grad_norms_clipped


def train_epoch(model, optimizer, baseline, lr_scheduler, epoch, val_dataset, problem, tb_logger, opts):
    cprint("Start train epoch {}, lr={} for run {}".format(epoch, optimizer.param_groups[0]['lr'], opts.run_name), 'red', attrs = ['bold'])
    step = epoch * (opts.epoch_size // opts.batch_size)
    
    
    if not opts.no_tensorboard:
        tb_logger.log_value('learnrate_pg0', optimizer.param_groups[0]['lr'], step)

    # Generate new training data for each epoch
    
    training_dataset = baseline.wrap_dataset(problem.make_dataset(
        uav_graph_size = opts.uav_graph_size, ugv_graph_size = opts.ugv_graph_size,  num_samples = opts.epoch_size))
    
    training_dataloader = DataLoader(training_dataset, batch_size=opts.batch_size, num_workers=0)
    
   
    # Put model in train mode!
    model.train()
    set_decode_type(model, "sampling")
    
    
    start_time = time.time()
    for batch_id, batch in enumerate(tqdm(training_dataloader, disable=opts.no_progress_bar)):
        
        eps_start_time = time.time()
        train_batch(
            model,
            optimizer,
            baseline,
            epoch,
            batch_id,
            step,
            batch,
            tb_logger,
            opts
        )

        step += 1
        print('\n Episode time ----->',time.time() - eps_start_time)

    epoch_duration = time.time() - start_time
    print('\n Epoch time --------->', epoch_duration)
    print("Finished epoch {}, took {} s".format(epoch, time.strftime('%H:%M:%S', time.gmtime(epoch_duration))))

    if (opts.checkpoint_epochs != 0 and epoch % opts.checkpoint_epochs == 0) or epoch == opts.n_epochs - 1:
        print('Saving model and state...')
        torch.save(
            {
                'model': get_inner_model(model).state_dict(),
                'optimizer': optimizer.state_dict(),
                'rng_state': torch.get_rng_state(),
                'cuda_rng_state': torch.cuda.get_rng_state_all(),
                'baseline': baseline.state_dict()
            },
            os.path.join(opts.save_dir, 'epoch-{}.pt'.format(epoch))
        )

    avg_reward = validate(model, val_dataset, opts)

    if not opts.no_tensorboard:
        tb_logger.log_value('val_avg_reward', avg_reward, step)

    baseline.epoch_callback(model, epoch)

    # lr_scheduler should be called at end of epoch
    lr_scheduler.step()


def train_batch(
        model,
        optimizer,
        baseline,
        epoch,
        batch_id,
        step,
        batch,
        tb_logger,
        opts
):
    #print(baseline)
    
    x, bl_val = baseline.unwrap_batch(batch)
    x = move_to(x, opts.device)
    
    #print('\n Bl val: \n', bl_val)
    
    bl_val = move_to(bl_val, opts.device) if bl_val is not None else None

    # Evaluate model, get costs and log probabilities
    
    cost, log_likelihood = model(x)
    print('requires grad --->', cost.requires_grad, log_likelihood.requires_grad)
    cprint(' batch 1 score in a episode ------> {} '.format(cost[0]), 'green', attrs = ['bold'] )
    cprint('Avg batch score in a episode ------> {} '.format(torch.mean(cost)), 'blue', attrs = ['bold'] )
    

    # Evaluate baseline, get baseline loss if any (only for critic)
    bl_val, bl_loss = baseline.eval(x, cost) if bl_val is None else (bl_val, 0)
    print('Baseline value:', bl_val.shape)
    
    print('log likelihood -->', cost.mean(), bl_val.mean(), log_likelihood.mean() )
    
    # Calculate loss
    reinforce_loss = ((cost - bl_val) * log_likelihood).mean()
    loss = reinforce_loss + bl_loss
    
    if torch.isnan(loss).any() or torch.isinf(loss).any():
       print("NaN or Inf detected in loss before backward")
       kkkkk

    # Perform backward pass and optimization step
    optimizer.zero_grad()
    loss.backward()
    for name, param in model.named_parameters():
       if param.grad is not None and torch.isnan(param.grad).any():
          print(f"NaN gradient in parameter: {name}")
          kkkkk
    # Clip gradient norms and get (clipped) gradient norms for logging
    grad_norms = clip_grad_norms(optimizer.param_groups, opts.max_grad_norm)
    optimizer.step()
    
    


    csv_file_path = 'training_log.csv'

    
    if not os.path.isfile(csv_file_path) or os.stat(csv_file_path).st_size == 0:
     with open(csv_file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Epoch', 'Batch_ID', 'Step', 'Cost', 'Log_Likelihood', 'Reinforce_Loss', 'BL_Loss'])


    if not opts.no_tensorboard:
        tb_logger.log_value('loss', loss, step)
        
    if not opts.no_tensorboard:
        tb_logger.log_value('bl_val', bl_val.mean().item(), step)
    
    if not opts.no_tensorboard:
        tb_logger.log_value('bl_loss', bl_loss, step)

    
    # Logging 
    if step % int(opts.log_step) == 0:
        log_values(cost, grad_norms, epoch, batch_id, step,
                   log_likelihood, reinforce_loss, bl_loss, tb_logger, opts)
        
        '''with open(csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([epoch, batch_id, step, cost[0] , log_likelihood, reinforce_loss, bl_loss])'''
