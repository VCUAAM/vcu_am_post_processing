import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def load_confusion_matrix(path):
    return np.load(path)

def remove_background(cm):
    # YOLOv5 confusion matrix includes background as last row/column
    return cm[:-1, :-1]

def remove_unused_classes(cm, keep_indices):
    """
    keep_indices: list of class indices to keep (e.g., [0, 1])
    """
    return cm[np.ix_(keep_indices, keep_indices)]

def plot_confusion_matrix_yolov5_style(
    cm,
    class_names,
    normalize=True,
    save_path=None
):
    """
    cm: confusion matrix with background and unused classes already removed
    class_names: list of class labels (length must match cm)
    """

    fig, ax = plt.subplots(figsize=(6, 6))

    if normalize:
        cm = cm.astype(float)
        row_sums = cm.sum(axis=1, keepdims=True)
        cm = np.divide(cm, row_sums, where=row_sums != 0)

    im = ax.imshow(cm, cmap='Blues')

    # Colorbar (YOLOv5 includes this)
    cbar = fig.colorbar(im, ax=ax)
    cbar.ax.set_ylabel('Normalized Count' if normalize else 'Count', rotation=90)

    ax.set_xlabel('Predicted')
    ax.set_ylabel('True')

    ax.set_xticks(range(len(class_names)))
    ax.set_yticks(range(len(class_names)))
    ax.set_xticklabels(class_names, rotation=45, ha='right')
    ax.set_yticklabels(class_names)

    # Annotate cells
    for i in range(cm.shape[0]):
        for j in range(cm.shape[1]):
            value = cm[i, j]
            if normalize:
                label = f"{value:.2f}" if value > 0 else ""
            else:
                label = f"{int(value)}" if value > 0 else ""

            ax.text(
                j,
                i,
                label,
                ha="center",
                va="center",
                color="white" if value > 0.5 else "black"
            )

    ax.set_aspect('equal')
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300)
    plt.show()

def main():
    cm_path = Path("ml_vision/data/confusion_matrix.npy")
    class_names = ["Class A", "Class B"]  # update to your real names

    cm = load_confusion_matrix(cm_path)
    cm = remove_background(cm)
    cm = remove_unused_classes(cm, keep_indices=[0, 1])

    plot_confusion_matrix_yolov5_style(
        cm,
        class_names=class_names,
        normalize=True,
        save_path="confusion_matrix_cleaned.png"
    )

if __name__ == "__main__":
    main()