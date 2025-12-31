import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np

plt.rcParams.update({'font.size': 16})

def load_results(csv_path):
    df = pd.read_csv(csv_path)
    # Strip whitespace from column names (YOLOv5 sometimes includes leading spaces)
    df.columns = df.columns.str.strip()
    return df

def plot_map_metrics(df, save_dir=None):
    plt.figure()
    plt.plot(df['epoch'], df['metrics/mAP_0.5'], label='mAP@0.5')
    plt.plot(df['epoch'], df['metrics/mAP_0.5:0.95'], label='mAP@0.5:0.95')
    plt.xlabel('Epoch')
    plt.ylabel('mAP')
    plt.legend()
    plt.xlim(0,17)
    plt.ylim(0,1)
    plt.tight_layout()

    if save_dir:
        plt.savefig(Path(save_dir) / 'mAP.png', dpi=300)
    plt.show()

def plot_precision_recall(df, save_dir=None):
    plt.figure()
    plt.plot(df['metrics/recall'], df['metrics/precision'])
    plt.xlabel('Recall')
    plt.ylabel('Precision')
    plt.xlim(0,1)
    plt.ylim(0,1)
    plt.tight_layout()

    if save_dir:
        plt.savefig(Path(save_dir) / 'precision_vs_recall.png', dpi=300)
    plt.show()

def plot_f1_score(df, save_dir=None):
    precision = df['metrics/precision']
    recall = df['metrics/recall']

    # Avoid division by zero
    f1 = 2 * (precision * recall) / (precision + recall)
    f1 = f1.fillna(0.0)
    f1_scores = np.array(f1)
    print(f1_scores)
    plt.figure()
    plt.plot(df['epoch'], f1, label=f'F1 Score = {round(max(f1_scores),4)}')
    plt.xlabel('Epoch')
    plt.ylabel('F1')
    plt.xlim(0,17)
    plt.ylim(0,1)
    plt.legend()
    plt.tight_layout()

    if save_dir:
        plt.savefig(Path(save_dir) / 'f1.png', dpi=300)
    plt.show()

def main():
    csv_path = 'ml_vision/data/results.csv'  # update if needed
    save_dir = 'ml_vision/data'        # set to None if you do not want to save figures

    df = load_results(csv_path)

    if save_dir:
        Path(save_dir).mkdir(exist_ok=True)

    plot_map_metrics(df, save_dir)
    plot_f1_score(df,save_dir)
    plot_precision_recall(df, save_dir)
    plot_losses(df, save_dir)

if __name__ == '__main__':
    main()