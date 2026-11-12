import torch
from torch import nn
from torch.distributions import Normal
class Policy(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor=nn.Sequential(nn.Linear(12,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh(),nn.Linear(64,2))
        self.critic=nn.Sequential(nn.Linear(12,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh(),nn.Linear(64,1))
        self.logstd=nn.Parameter(torch.full((2,),-.7))
        nn.init.orthogonal_(self.actor[-1].weight,.01);nn.init.zeros_(self.actor[-1].bias)
    def forward(self,obs):
        return Normal(self.actor(obs),self.logstd.exp()),self.critic(obs).squeeze(-1)
