"""
Object tracking utilities using SORT (Simple Online Realtime Tracking).
"""

import torch
from scipy.optimize import linear_sum_assignment
from torchvision.ops import box_iou


class KalmanFilterTorch:
    """Kalman filter implementation in PyTorch for 2D position tracking."""
    
    def __init__(self, dt: float = 1.0):
        """
        Initialize Kalman filter.
        
        Parameters:
            dt: Time step for state transition (default: 1.0).
        """
        self.dt = dt
        self._update_matrices(dt)
        self.Q = torch.eye(4) * 0.01  # Process noise covariance
        self.R = torch.eye(2) * 0.1   # Measurement noise covariance

    def _update_matrices(self, dt: float):
        """Update state transition matrices with new time step."""
        # State transition embeds dt in the constant-velocity model
        self.F = torch.tensor([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=torch.float32)
        
        self.H = torch.tensor([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=torch.float32)

    def predict(self, x: torch.Tensor, P: torch.Tensor, dt: float = None):
        """
        Predict next state.
        
        Parameters:
            x: State vector [x, y, vx, vy].
            P: State covariance matrix.
            dt: Optional new time step.
        
        Returns:
            Tuple of (predicted_state, predicted_covariance).
        """
        if dt is not None and dt != self.dt:
            self.dt = dt
            self._update_matrices(dt)
        
        x_pred = self.F @ x
        P_pred = self.F @ P @ self.F.t() + self.Q
        return x_pred, P_pred

    def update(self, x_pred: torch.Tensor, P_pred: torch.Tensor, z: torch.Tensor):
        """
        Update state with measurement.
        
        Parameters:
            x_pred: Predicted state vector.
            P_pred: Predicted covariance matrix.
            z: Measurement vector [x, y].
        
        Returns:
            Tuple of (updated_state, updated_covariance).
        """
        y = z - (self.H @ x_pred)
        S = self.H @ P_pred @ self.H.t() + self.R
        K = P_pred @ self.H.t() @ torch.inverse(S)
        x_upd = x_pred + K @ y
        P_upd = (torch.eye(4) - K @ self.H) @ P_pred
        return x_upd, P_upd


class SortTrack:
    """
    SORT tracker for multi-object tracking with class-aware matching.
    """
    
    def __init__(self, max_age=1, min_hits=3, iou_thr=0.3, guard=[0]):
        """
        Initialize SORT tracker.
        
        Parameters:
            max_age: Maximum frames to keep track without update.
            min_hits: Minimum hits before track is confirmed.
            iou_thr: IoU threshold for matching detections to tracks.
            guard: List of guard factors per class for bounding box expansion.
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_thr = iou_thr
        self.guard = guard
        self.trackers = []
        self.next_id = 0

    def _state_to_bbox(self, trk):
        """Convert tracker state to bounding box coordinates."""
        cx, cy, vx, vy = trk['x']
        w, h = trk['w'], trk['h']
        # Convert center (cx, cy) with width/height to [x1, y1, x2, y2]
        return torch.tensor([cx - w/2, cy - h/2, cx + w/2, cy + h/2], dtype=torch.float32)

    def update(self, dets, classes, dt):
        """
        Update tracks with new detections.
        
        Parameters:
            dets: Tensor of N x 4 [x1, y1, x2, y2] detections.
            classes: Tensor of N class ids for each detection.
            dt: Time delta since last update.
        
        Returns:
            Tuple of (predictions, mappings, any_confirmed):
                - predictions: Tensor of predicted boxes [x1, y1, x2, y2, track_id, class_id]
                - mappings: Tensor of [detection_idx, track_id] pairs
                - any_confirmed: Boolean indicating if any tracks were matched
        """
        preds = torch.empty((0, 6))
        
        # 1) Predict all trackers forward one timestep
        for trk in self.trackers:
            trk['x'], trk['P'] = trk['kf'].predict(trk['x'], trk['P'], dt)
            trk['age'] += 1
            bbox = self._state_to_bbox(trk)
            preded = torch.tensor([
                bbox[0].item(), bbox[1].item(), bbox[2].item(), bbox[3].item(), 
                trk['id'], trk.get("class_id", -1)
            ])
            preds = torch.vstack([preds, preded.unsqueeze(0)])

        # 2) Build the (M×4) tensor of predicted bboxes
        if self.trackers:
            trks = torch.stack([self._state_to_bbox(trk) for trk in self.trackers], dim=0)
        else:
            trks = torch.empty((0, 4))
            
        # 3) Compute IoU matrix and perform assignment
        if trks.numel():
            iou_mat = box_iou(dets, trks).cpu().numpy()  # shape: (N, M)
            row_ind, col_ind = linear_sum_assignment(-iou_mat)
            matches, unmatched_d, unmatched_t = [], set(), set()
            
            for d, t in zip(row_ind, col_ind):
                # Check IoU threshold and class match
                if (iou_mat[d, t] >= self.iou_thr and 
                    classes[d].item() == self.trackers[t].get("class_id", None)):
                    matches.append((d, t))
                else:
                    unmatched_d.add(d)
                    unmatched_t.add(t)
            
            unmatched_d |= set(range(dets.size(0))) - {d for d, _ in matches}
            unmatched_t |= set(range(trks.size(0))) - {t for _, t in matches}
        else:
            matches, unmatched_d, unmatched_t = [], set(range(dets.size(0))), set()

        mappings = torch.empty((0, 2), dtype=torch.long)
        
        # 4) Update matched tracks
        for d, t in matches:
            # Use the center of the detection box
            z = torch.tensor([
                (dets[d, 0] + dets[d, 2]) / 2, 
                (dets[d, 1] + dets[d, 3]) / 2
            ], device=dets.device, dtype=torch.float32)
            
            trk = self.trackers[t]
            trk['x'], trk['P'] = trk['kf'].update(trk['x'], trk['P'], z)
            trk['hits'] += 1
            trk['age'] = 0
            
            # Update tracked width and height
            det_w = dets[d, 2] - dets[d, 0]
            det_h = dets[d, 3] - dets[d, 1]
            avg_dim = (det_w + det_h) / 2
            guard_factor = self.guard[classes[d].item()]
            size = avg_dim * (1 + guard_factor)
            trk['w'] = size.item()
            trk['h'] = size.item()
            
            mappings = torch.vstack([mappings, torch.tensor([d, trk['id']]).unsqueeze(0)])

        # 5) Create new tracks for unmatched detections
        for d in unmatched_d:
            base_w = dets[d, 2] - dets[d, 0]
            base_h = dets[d, 3] - dets[d, 1]
            avg_dim = (base_w + base_h) / 2
            cls = classes[d].item()
            guard_factor = self.guard[cls]
            size = avg_dim * (1 + guard_factor)
            w, h = size, size
            
            # Use center of box
            x = (dets[d, 0] + dets[d, 2]) / 2
            y = (dets[d, 1] + dets[d, 3]) / 2
            kf = KalmanFilterTorch()
            x0 = torch.tensor([x, y, 0, 0], dtype=torch.float32)
            
            self.trackers.append({
                'kf': kf, 'x': x0, 'P': torch.eye(4),
                'hits': 1, 'age': 0,
                'w': w.item(), 'h': h.item(),
                'id': self.next_id,
                'class_id': cls
            })
            mappings = torch.vstack([mappings, torch.tensor([d, self.next_id]).unsqueeze(0)])
            self.next_id += 1
            
        # 6) Age and prune tracks
        for trk in self.trackers:
            trk['age'] += 1
        self.trackers = [t for t in self.trackers if t['age'] <= self.max_age]

        return preds, mappings, len(mappings) > 0
