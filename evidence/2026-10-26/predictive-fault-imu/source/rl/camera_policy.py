import torch
from torch import nn
from torch.distributions import Normal
class CameraPolicy(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor=nn.Sequential(nn.Linear(6,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh(),nn.Linear(64,1))
        self.critic=nn.Sequential(nn.Linear(6,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh(),nn.Linear(64,1))
        self.logstd=nn.Parameter(torch.full((1,),-.7))
        nn.init.orthogonal_(self.actor[-1].weight,.01);nn.init.zeros_(self.actor[-1].bias)
    def forward(self,x):return Normal(self.actor(x),self.logstd.exp()),self.critic(x).squeeze(-1)
