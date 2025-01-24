## Testing Inverse kineto-static (IKS) Network

### Old model + brush

1. Generate the circle trajectory $X,Q$
2. Seting origin using OLD home position: $X_s = X + X_{s,home}$
3. Neural network inference: $X_s,Q_s \rightarrow \mathcal{L}$
    * Need to intialize bot_DNN with flag weighted = False
4. Record the actual positions: $\hat{X}_t,\hat{Q}_t$
5. Apply the batch transform to obtain $\hat{X}_s,\hat{Q}_s$
5. Calculate the $L_2$ norm $\| \hat{X_s} - X_s \|_2$

### New model + brush

1. Generate the circle trajectory $X,Q, w$.
2. Set the origin using NEW home position: $X_t = X + X_{t,home}$
2. Run the new model inference: $X_t,Q_t, w \rightarrow \mathcal{L}$
4. Record the actual positions: $\hat{X_t},\hat{Q_t}$
5. Calculate the $L_2$ norm $\| \hat{X_t} - X_t \|_2$
