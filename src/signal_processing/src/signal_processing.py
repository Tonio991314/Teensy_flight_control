#!/usr/bin/env python

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as mpatches
import argparse
# import os

def find_round(df):
    #change datatype to int
    columns_to_convert = df.columns[:6]
    for col in columns_to_convert:
        df[col] = df[col].astype(int)

    round =[]
    count=0
    record=[0,0,0]
    for row in range (df.shape[0]):
        if row != 0:
            temp=df.iloc[row-1][5]
        else:
            temp=0
        #record round, start, and end rows
        if df.iloc[row][5] >= 1000 and temp < 980 :
            record=[count+1, row, 0]
        elif df.iloc[row][5] <= 1000 and temp > 980 :
            record[2]=row
            round.append(record)
            count+=1
    print("There are "+str(count)+" rounds in this test")
    return round

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("file_name", help="Please enter the file name without extension", type=str)
    args = parser.parse_args()
    input_file_name = args.file_name


    input_file_path = "./log_file/"+input_file_name+".txt"
    output_file_path1 = "./temp_txt/rm_key.txt"
    output_file_path2 = "./temp_txt/rm_key_rc.txt"
    # target = ["0", "Q"]
    target = ["0", "4", "5", "2", "6", "8", "9", "Q"]
    target_key=["4", "5", "2", "6", "8", "9"]
    # Read input data from the input file
    with open(input_file_path, 'r') as input_file:
        input_data = input_file.read()

    # Split the input data into lines
    lines = input_data.strip().split('\n')

    # Initialize an empty list to store non-empty lines
    filtered_lines = []
    total=0
    del_count=0
    timeframe=[]
    key=[]
    # Iterate through each line and filter out lines that match any element from the target list
    for line in lines:
        total+=1
        if line.strip() not in target:
            filtered_lines.append(line)
        if line.strip() in target:
            del_count+=1
            for x in line.strip():
                for y in target_key:
                    if x==y:
                        del_count+=1
                        timeframe.append(total-del_count)
                        key.append(x)

    # Join the filtered lines back together
    output_data = '\n'.join(filtered_lines)

    # Save the filtered output to the output file
    with open(output_file_path1, 'w') as output_file:
        output_file.write(output_data)

    print("Key removed data saved to"+ str(output_file_path1))

    with open(output_file_path1, 'r') as input_file:
        lines = input_file.readlines()

    # Remove text from "Channel 7: " to "Channel 12: " and write to the output file
    with open(output_file_path2, 'w') as output_file:
        for line in lines:
            modified_line = line.replace('Channel 7: ', '').replace('Channel 8: ', '').replace('Channel 9: ', '') \
                            .replace('Channel 10: ', '').replace('Channel 11: ', '').replace('Channel 12: ', '')
            # Add more .replace() calls as needed for other channels
            output_file.write(modified_line)

    print("Key & Channel removed data saved to "+str(output_file_path2))
    print("===File Saved!===")

    df = pd.read_csv(output_file_path1, delimiter='/')
    df_new = pd.read_csv(output_file_path2, delimiter='/',header=None)

    channel_dict={'4': [2],'5': [1,2],'6': [2],'8': [1],'2': [1],'1': [1,2],'7': [1,2], '9':[1,2], '3':[1,2]}
    color_names = {'1':'red','2':'green','3': 'blue','4': 'orange','5': 'purple','6': 'brown','7': 'pink', '8':'gray', '9':'cyan'}
    # Create a proxy artist for the legend with a specific color
    legend_left = mpatches.Patch(color=color_names["4"], label='Left(4)')
    legend_stay = mpatches.Patch(color=color_names["5"], label='Stay(5)')
    legend_right = mpatches.Patch(color=color_names["6"], label='Right(6)')
    legend_forward = mpatches.Patch(color=color_names["8"], label='Forward(8)')
    legend_backward = mpatches.Patch(color=color_names["2"], label='Backward(2)')
    legend_FL = mpatches.Patch(color=color_names["7"], label='Front-Left(7)')
    legend_FR = mpatches.Patch(color=color_names["9"], label='Front-Right(9)')
    legend_BL = mpatches.Patch(color=color_names["1"], label='Back-Left(1)')
    legend_BR = mpatches.Patch(color=color_names["3"], label='Back-Right(3)')


    X_ch=[]
    Y_ch=[]
    y_range=np.linspace(1400, 1700, 15)
    rounds=find_round(df_new)

    for round in range(len(rounds)):
        fig, axes = plt.subplots(nrows=6, ncols=1, figsize=(10, 20))
        start=rounds[round][1]
        end=rounds[round][2]
        for channel in range(1, 7):
            X=[]
            Y=[]
            for idx in range(start , end):
                X.append(idx)
                Y.append(int(df[df.columns[channel-1]][idx][-5:-1]))
            X_ch.append(X)
            Y_ch.append(Y)
            axes[channel-1].plot(X, Y)
            axes[channel-1].set_title("Channel "+ str(channel))
            if channel in [1,2,4]:
                y_range=np.linspace(1300, 1700, 15)
                axes[channel-1].set_yticks(y_range)
            elif channel in [3]:
                y_range=np.linspace(800, 1700, 15)
                axes[channel-1].set_yticks(y_range)
            elif channel in [5]:
                y_range=np.linspace(2200, 2400, 15)
                axes[channel-1].set_yticks(y_range)
            elif channel in [6]:
                y_range=np.linspace(800, 2350, 15)
                axes[channel-1].set_yticks(y_range)
                
        axes[3].legend(handles=[legend_left, legend_stay ,legend_right , legend_forward, legend_backward, legend_FL, legend_FR, legend_BL, legend_BR], bbox_to_anchor=(1, 5.5))


        x=[]
        y=[]
        size=8
        count=0
        for idx in range(len(timeframe)):
            # print("timeframe[idx] :",timeframe[idx] , "start: ", start)
            # print("timeframe[idx] :",timeframe[idx] , "end: ", end)
            if timeframe[idx] >= start and timeframe[idx] <= end:
                count+=1
                if key[idx]== "5" or key[idx]== "7" or key[idx]== "9" or key[idx]== "1" or key[idx]== "3" :
                    axes[0].scatter(x=timeframe[idx], y=df_new.iloc[timeframe[idx]][0], color = color_names[str(key[idx])], s=size)
                    axes[1].scatter(x=timeframe[idx], y=df_new.iloc[timeframe[idx]][1], color = color_names[str(key[idx])], s=size)
                elif key[idx]== "8":
                    axes[0].scatter(x=timeframe[idx], y=df_new.iloc[timeframe[idx]][0], color = color_names[str(key[idx])], s=size)
                elif key[idx]== "2":
                    axes[0].scatter(x=timeframe[idx], y=df_new.iloc[timeframe[idx]][0], color = color_names[str(key[idx])], s=size)
                elif key[idx]== "4":
                    axes[1].scatter(x=timeframe[idx], y=df_new.iloc[timeframe[idx]][1], color = color_names[str(key[idx])], s=size)
                elif key[idx]== "6":
                    axes[1].scatter(x=timeframe[idx], y=df_new.iloc[timeframe[idx]][1], color = color_names[str(key[idx])], s=size)

            for channel in range(1, 7):
                if channel in [1,2,4]:
                    y_range=np.linspace(1300, 1700, 15)
                    axes[channel-1].set_yticks(y_range)
                elif channel in [3]:
                    y_range=np.linspace(800, 1700, 15)
                    axes[channel-1].set_yticks(y_range)
                elif channel in [5]:
                    y_range=np.linspace(800, 2400, 15)
                    axes[channel-1].set_yticks(y_range)
                elif channel in [6]:
                    y_range=np.linspace(800, 2350, 15)
                    axes[channel-1].set_yticks(y_range)
                else:
                    pass
            
        print("Round " + str(round+1) + " in loop: "+ str(count))
        suptitle = fig.suptitle('Round ' + str(round+1))
        suptitle.set_fontsize(40)
        # #save figure
        fig_name="_round_"+str(round+1)+".png"
        fig.savefig("./signal_picture/"+input_file_name+fig_name, format='png', dpi=500)
        
    print("===Figure saved!===")
    
