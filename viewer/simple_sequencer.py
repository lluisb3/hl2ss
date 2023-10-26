#------------------------------------------------------------------------------
# Sequencer example. The sequencer associates data from two or more files.
# Here, it is used to pair RM VLC LEFTFRONT and RIGHTFRONT frames with PV 
# frames, all previously recorded using simple recorder.
#------------------------------------------------------------------------------

import cv2
import hl2ss_imshow
import hl2ss
import hl2ss_io
from pathlib import Path

thispath = Path(__file__).resolve()


# Settings --------------------------------------------------------------------
exp_name = 'pointcloud'

# Directory containing the recorded data
path = f'{thispath.parent.parent}/data/{exp_name}'

#------------------------------------------------------------------------------

# Create readers --------------------------------------------------------------
rd_pv = hl2ss_io.create_rd(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.PERSONAL_VIDEO)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, 'bgr24')
rd_lf = hl2ss_io.sequencer(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.RM_VLC_LEFTFRONT)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, True)
rd_rf = hl2ss_io.sequencer(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.RM_VLC_RIGHTFRONT)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, True)
rd_depth = hl2ss_io.sequencer(f'{path}/{hl2ss.get_port_name(hl2ss.StreamPort.RM_DEPTH_LONGTHROW)}.bin', hl2ss.ChunkSize.SINGLE_TRANSFER, True)

# Open readers ----------------------------------------------------------------
rd_pv.open()
rd_lf.open()
rd_rf.open()
rd_depth.open()

# Main loop -------------------------------------------------------------------
while (True):
    # Get PV frame ------------------------------------------------------------
    data_pv = rd_pv.get_next_packet()
    if (data_pv is None):
        break

    # Find RM VLC frames corresponding to the current PV frame ----------------
    data_lf = rd_lf.get_next_packet(data_pv.timestamp) # Get nearest (in time) lf frame
    data_rf = rd_rf.get_next_packet(data_pv.timestamp) # Get nearest (in time) rf frame
    data_depth = rd_depth.get_next_packet(data_pv.timestamp)

    print(data_depth)

    # Display frames ----------------------------------------------------------
    if (data_lf is not None):
        cv2.imshow('RM VLC LF', data_lf.payload)
    if (data_rf is not None):
        cv2.imshow('RM VLC RF', data_rf.payload)
    if (data_depth is not None):
        cv2.imshow('RM Depth', data_depth.payload.depth)

    cv2.imshow('PV', data_pv.payload.image)
    cv2.waitKey(1)

# Close readers ---------------------------------------------------------------
rd_pv.close()
rd_lf.close()
rd_rf.close()
rd_depth.close()
