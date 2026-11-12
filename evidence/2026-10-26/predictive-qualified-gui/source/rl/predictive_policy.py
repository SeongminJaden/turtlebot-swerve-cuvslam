import torch
from torch import nn
from torch.distributions import Normal
class PredictivePolicy(nn.Module):
 def __init__(self):
  super().__init__();self.actor=nn.Sequential(nn.Linear(24,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh(),nn.Linear(64,2));self.critic=nn.Sequential(nn.Linear(24,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh(),nn.Linear(64,1));self.logstd=nn.Parameter(torch.tensor([-.7,-1.]));nn.init.orthogonal_(self.actor[-1].weight,.01);nn.init.zeros_(self.actor[-1].bias)
 def initialize_camera(self,state):
  with torch.no_grad():
   self.actor[0].weight.zero_();self.actor[0].weight[:,:6]=state['actor.0.weight'];self.actor[0].bias.copy_(state['actor.0.bias']);self.actor[2].weight.copy_(state['actor.2.weight']);self.actor[2].bias.copy_(state['actor.2.bias']);self.actor[4].weight[1]=state['actor.4.weight'][0];self.actor[4].bias[1]=state['actor.4.bias'][0];self.actor[4].bias[0]=.2
 def forward(self,x):return Normal(self.actor(x),self.logstd.exp()),self.critic(x).squeeze(-1)
