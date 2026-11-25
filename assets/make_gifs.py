"""Regenerate the GIFs/PNGs of the README.

Python port of the MATLAB functions of this repo (forward_kinematics,
compute_jacobian, inverse_kinematics_inverse / _transpose), used only to
render the animations without needing a MATLAB licence.

    pip install numpy matplotlib pillow
    python assets/make_gifs.py
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

OUT = Path(__file__).parent
L = np.array([1.0, 1.0])
FPS = 25


# ---------------------------------------------------------------- MATLAB port
def forward_kinematics(q, L):
    t1, t12 = q[0], q[0] + q[1]
    p1 = L[0] * np.array([np.cos(t1), np.sin(t1)])
    p2 = p1 + L[1] * np.array([np.cos(t12), np.sin(t12)])
    return p2, np.column_stack([[0, 0], p1, p2])


def compute_jacobian(q, L):
    r, P = forward_kinematics(q, L)
    # same convention as the MATLAB code: its p1/p2 are P(:,1)/P(:,2), i.e.
    # the base and the first joint (rotation axes of joints 1 and 2)
    p1, p2 = P[:, 0], P[:, 1]
    e1, e2 = r - p1, r - p2
    return np.column_stack([[-e1[1], e1[0]], [-e2[1], e2[0]]])


def solve(J, v):
    """MATLAB's J\\v: falls back to least squares when J is singular."""
    if abs(np.linalg.det(J)) < 1e-12:
        return np.linalg.lstsq(J, v, rcond=None)[0]
    return np.linalg.solve(J, v)


def ik_inverse(q0, r_d, L, k=1.0, dt=0.05, eps=1e-4, N=1000):
    q, Q, err = q0.copy(), [q0.copy()], []
    for _ in range(N):
        r, _ = forward_kinematics(q, L)
        err.append(np.linalg.norm(r_d - r))
        if err[-1] < eps:
            break
        q = q + solve(compute_jacobian(q, L), k * (r_d - r)) * dt
        Q.append(q.copy())
    return np.array(Q).T, np.array(err)


def ik_transpose(q0, r_d, L, eta=1.0, C=(1, 1), dt=0.05, eps=1e-4, N=1000):
    K = np.diag(1.0 / np.asarray(C, float))
    q, Q, err = q0.copy(), [q0.copy()], []
    for _ in range(N):
        r, _ = forward_kinematics(q, L)
        err.append(np.linalg.norm(r_d - r))
        if err[-1] < eps:
            break
        q = q + K @ (compute_jacobian(q, L).T @ (eta * (r_d - r))) * dt
        Q.append(q.copy())
    return np.array(Q).T, np.array(err)


# --------------------------------------------------------------------- render
def new_axes(title, lim=None, size=(4.2, 4.2)):
    lim = lim or L.sum() * 1.1
    fig, ax = plt.subplots(figsize=size, dpi=80)
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_title(title, fontsize=10)
    # reachable workspace
    ax.add_patch(plt.Circle((0, 0), L.sum(), fill=False, ls=":", color="gray"))
    return fig, ax


def subsample(n, max_frames=110):
    step = max(1, int(np.ceil(n / max_frames)))
    idx = list(range(0, n, step))
    if idx[-1] != n - 1:
        idx.append(n - 1)
    return idx + [n - 1] * 12  # hold last frame


def save(fig, update, frames, name):
    anim = FuncAnimation(fig, update, frames=frames, blit=False)
    anim.save(OUT / name, writer=PillowWriter(fps=FPS))
    plt.close(fig)
    print("wrote", name)


def arm_gif(Q, name, title, r_d=None, lim=None):
    fig, ax = new_axes(title, lim)
    if r_d is not None:
        ax.plot(*r_d, "kx", ms=12, mew=2)
    (arm,) = ax.plot([], [], "o-", lw=3, ms=8, color="C0")
    (trace,) = ax.plot([], [], "r--", lw=1.2)
    tip = np.array([forward_kinematics(Q[:, i], L)[0] for i in range(Q.shape[1])]).T
    idx = subsample(Q.shape[1])

    def update(f):
        i = idx[f]
        arm.set_data(*forward_kinematics(Q[:, i], L)[1])
        trace.set_data(tip[0, : i + 1], tip[1, : i + 1])

    save(fig, update, len(idx), name)


# ------------------------------------------------------------------- the GIFs
def gif_forward():
    N = 200
    Q = np.vstack([np.linspace(0, np.pi / 2, N)] * 2)
    arm_gif(Q, "01_forward_kinematics.gif", "Forward kinematics: θ1, θ2: 0 → π/2")


def gif_jacobian_directions():
    q0, dt, N = np.array([np.pi / 2, np.pi / 2]), 0.01, 50
    r0, _ = forward_kinematics(q0, L)
    fig, ax = new_axes("Jacobian: 10 desired velocity directions", lim=2.2)
    colors = plt.cm.tab10(np.arange(10))
    trajs = []
    for i, th in enumerate((np.arange(10)) * np.pi / 5):
        v, q, T = np.array([np.cos(th), np.sin(th)]), q0.copy(), [r0]
        qs = [q0.copy()]
        for _ in range(N):
            q = q + dt * solve(compute_jacobian(q, L), v)
            T.append(forward_kinematics(q, L)[0])
            qs.append(q.copy())
        trajs.append((np.array(T).T, np.array(qs).T))
    lines = [ax.plot([], [], "-", lw=2, color=colors[i])[0] for i in range(10)]
    (arm,) = ax.plot([], [], "ko-", lw=3, ms=7)
    frames = [(i, k) for i in range(10) for k in range(1, N + 1, 2)]
    frames += [(9, N)] * 12

    def update(f):
        i, k = frames[f]
        T, qs = trajs[i]
        lines[i].set_data(T[0, : k + 1], T[1, : k + 1])
        arm.set_data(*forward_kinematics(qs[:, k], L)[1])

    save(fig, update, len(frames), "02_jacobian_directions.gif")


def gif_ik(method, name, title, r_d, q0, lim=None):
    if method == "inv":
        Q, err = ik_inverse(q0, r_d, L)
    else:
        Q, err = ik_transpose(q0, r_d, L)
    arm_gif(Q, name, f"{title} ({len(err)} it.)", r_d, lim)
    return Q, err


def plot_convergence(err_inv, err_tr, name, title):
    fig, ax = plt.subplots(figsize=(5, 3.4), dpi=90)
    ax.semilogy(err_inv, "r", lw=2, label="Jacobian inverse")
    ax.semilogy(err_tr, "b", lw=2, label="Jacobian transpose")
    ax.grid(alpha=0.3, which="both")
    ax.set(xlabel="iteration", ylabel="|r_d − r|", title=title)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / name)
    plt.close(fig)
    print("wrote", name)


def plot_landscape(r_d, Q_inv, Q_tr, q0, name):
    t = np.linspace(-np.pi, np.pi, 200)
    T1, T2 = np.meshgrid(t, t)
    E = np.zeros_like(T1)
    for j in range(T1.size):
        E.flat[j] = np.linalg.norm(r_d - forward_kinematics([T1.flat[j], T2.flat[j]], L)[0])
    fig, ax = plt.subplots(figsize=(5.2, 4.4), dpi=90)
    cs = ax.contour(T1, T2, E, 25)
    fig.colorbar(cs)
    ax.plot(*Q_inv[:, :], "r-", lw=2.5, label="Jacobian inverse")
    ax.plot(*Q_tr[:, :], "b-", lw=2.5, label="Jacobian transpose")
    ax.plot(*q0, "ko", label="start")
    ax.plot(*Q_inv[:, -1], "kp", ms=13, mfc="y", label="end")
    ax.set(xlabel="θ1 [rad]", ylabel="θ2 [rad]", xlim=(-np.pi, np.pi), ylim=(-np.pi, np.pi),
           title="IK paths in joint space")
    ax.legend(fontsize=7, loc="lower left")
    fig.tight_layout()
    fig.savefig(OUT / name)
    plt.close(fig)
    print("wrote", name)


def gif_landscape_3d(r_d, Q_inv, Q_tr, q0, name):
    t = np.linspace(-np.pi, np.pi, 90)
    T1, T2 = np.meshgrid(t, t)
    E = np.array([np.linalg.norm(r_d - forward_kinematics([a, b], L)[0])
                  for a, b in zip(T1.ravel(), T2.ravel())]).reshape(T1.shape)
    err = lambda Q: np.array([np.linalg.norm(r_d - forward_kinematics(Q[:, i], L)[0])
                              for i in range(Q.shape[1])])
    fig = plt.figure(figsize=(5.4, 4.6), dpi=80)
    ax = fig.add_subplot(projection="3d")
    ax.plot_surface(T1, T2, E, cmap="viridis", alpha=0.75, linewidth=0, antialiased=True)
    for Q, c, lab in ((Q_inv, "r", "Jacobian inverse"), (Q_tr, "b", "Jacobian transpose")):
        ax.plot(Q[0], Q[1], err(Q) + 0.05, c, lw=2.5, label=lab)
    ax.scatter(*q0, np.linalg.norm(r_d - forward_kinematics(q0, L)[0]) + 0.05,
               c="k", s=30, label="start")
    ax.set(xlabel="θ1 [rad]", ylabel="θ2 [rad]", zlabel="|r_d − f(q)|")
    ax.set_title("Error landscape in joint space", fontsize=10)
    ax.legend(fontsize=7, loc="upper left")
    frames = 60

    def update(f):
        ax.view_init(elev=28, azim=-60 + 360 * f / frames)

    save(fig, update, frames, name)


if __name__ == "__main__":
    gif_forward()
    gif_jacobian_directions()

    # Q3: reachable target
    q0, r_d = np.array([np.pi / 6, np.pi / 3]), np.array([-1.0, 0.0])
    gif_ik("inv", "03_ik_inverse.gif", "IK: Jacobian inverse", r_d, q0)
    gif_ik("tr", "04_ik_transpose.gif", "IK: Jacobian transpose", r_d, q0)
    _, e_inv = ik_inverse(q0, r_d, L)
    _, e_tr = ik_transpose(q0, r_d, L)
    plot_convergence(e_inv, e_tr, "05_convergence.png", "Convergence, reachable target")

    # Q4: unreachable target (|r_d| = 3.6 > L1+L2 = 2) -> singularity
    q0s, r_ds = np.array([np.pi / 2, np.pi / 2]), np.array([2.0, 3.0])
    Qi, ei = gif_ik("inv", "06_singularity_inverse.gif", "Singularity: inverse", r_ds, q0s, lim=3.4)
    Qt, et = gif_ik("tr", "07_singularity_transpose.gif", "Singularity: transpose", r_ds, q0s, lim=3.4)
    plot_convergence(ei, et, "08_convergence_singularity.png", "Convergence, unreachable target")
    plot_landscape(r_ds, Qi, Qt, q0s, "09_joint_space_landscape.png")

    # 3D landscape on the reachable target (the singular case diverges in joint space)
    Qi3, _ = ik_inverse(q0, r_d, L)
    Qt3, _ = ik_transpose(q0, r_d, L)
    gif_landscape_3d(r_d, Qi3, Qt3, q0, "10_joint_space_landscape_3d.gif")
