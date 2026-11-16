// SPDX-License-Identifier: GPL-2.0-only
/*
 * V4L2 Pipeline Format Propagation Helper
 *
 * Copyright (C) 2026 Alif Semiconductor
 * Author: Yogender Kumar Arya <yogender.kumar@alifsemi.com>
 */

#include <linux/module.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>

#include "plat.h"

static bool is_sensor_entity(struct media_entity *entity)
{
	return (entity->function == MEDIA_ENT_F_CAM_SENSOR);
}

static int parse_camera_nodes(struct media_entity *start,
			      struct v4l2_subdev *sensors[], int *num_sensors)
{
	struct media_entity *entity;
	struct media_graph graph;
	int found = 0;
	int ret;

	ret = media_graph_walk_init(&graph, start->graph_obj.mdev);
	if (ret)
		return ret;

	media_graph_walk_start(&graph, start);

	while ((entity = media_graph_walk_next(&graph))) {
		if (entity->function != MEDIA_ENT_F_CAM_SENSOR)
			continue;

		if (is_media_entity_v4l2_subdev(entity))
			sensors[found++] = media_entity_to_v4l2_subdev(entity);

		if (found >= *num_sensors)
			break;
	}
	media_graph_walk_cleanup(&graph);
	*num_sensors = found;

	return 0;
}

static int apply_pad_format(struct v4l2_subdev *sd, int pad,
			    struct v4l2_subdev_format *fmt,
			    const struct v4l2_mbus_framefmt *mbus,
			    struct v4l2_subdev_state *state,
			    struct device *dev)
{
	struct v4l2_subdev_format entity_fmt;
	int ret;

	entity_fmt = *fmt;
	entity_fmt.pad = pad;
	entity_fmt.format = *mbus;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		struct v4l2_subdev_state *sd_state;

		sd_state = v4l2_subdev_lock_and_get_active_state(sd);
		ret = v4l2_subdev_call(sd, pad, set_fmt, sd_state, &entity_fmt);
		if (sd_state)
			v4l2_subdev_unlock_state(sd_state);
	} else {
		ret = v4l2_subdev_call(sd, pad, set_fmt, state, &entity_fmt);
	}

	if (ret && ret != -ENOIOCTLCMD) {
		dev_err(dev, "Failed to set format on %s pad %d: %d\n",
			sd->entity.name, pad, ret);
		return ret;
	}

	fmt->format = entity_fmt.format;
	return 0;
}

static int set_format_to_entity(struct media_entity *entity,
				struct v4l2_subdev_format *fmt,
				const struct v4l2_mbus_framefmt *requested,
				struct v4l2_subdev_state *state,
				struct device *dev)
{
	struct v4l2_subdev *sd;
	struct v4l2_mbus_framefmt incoming;
	int sink_pad = -1;
	int source_pad = -1;
	int ret;
	int i;

	if (!is_media_entity_v4l2_subdev(entity))
		return 0;

	sd = media_entity_to_v4l2_subdev(entity);
	incoming = fmt->format;

	for (i = 0; i < entity->num_pads; i++) {
		if (sink_pad < 0 && (entity->pads[i].flags & MEDIA_PAD_FL_SINK))
			sink_pad = i;
		if (source_pad < 0 &&
		    (entity->pads[i].flags & MEDIA_PAD_FL_SOURCE))
			source_pad = i;
	}

	if (is_sensor_entity(entity)) {
		if (source_pad < 0) {
			dev_err(dev, "Entity %s has no source pad\n",
				entity->name);
			return -EINVAL;
		}

		return apply_pad_format(sd, source_pad, fmt, &incoming,
					state, dev);
	}

	if (sink_pad >= 0) {
		ret = apply_pad_format(sd, sink_pad, fmt, &incoming,
				       state, dev);
		if (ret)
			return ret;
	}

	if (source_pad >= 0 && source_pad != sink_pad) {
		struct v4l2_mbus_framefmt source_fmt;

		source_fmt = fmt->format;
		source_fmt.code = requested->code;

		ret = apply_pad_format(sd, source_pad, fmt, &source_fmt,
				       state, dev);
		if (ret)
			return ret;
	} else if (sink_pad < 0) {
		dev_err(dev, "Entity %s has no valid pad for format setting\n",
			entity->name);
		return -EINVAL;
	}

	return 0;
}

/**
 * pipeline_set_format - Propagate format through entire media pipeline
 * @sd: Starting subdev (typically CPI)
 * @state: Subdev state (NULL for ACTIVE format)
 * @fmt: Format to propagate
 *
 * This function walks the media graph starting from sensors and propagates
 * the format forward through all entities in the pipeline.
 *
 * Returns 0 on success, negative error code on failure.
 */
int pipeline_set_format(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state,
			struct v4l2_subdev_format *fmt)
{
	struct v4l2_subdev *sensors[ALIF_MAX_CSI_SENSORS] = { NULL };
	struct media_entity *start;
	struct media_entity *entity;
	struct media_graph graph;
	struct media_entity *procs[ALIF_MAX_PIPELINE_PROCS];
	struct v4l2_mbus_framefmt requested;
	struct v4l2_mbus_framefmt sensor_fmt;
	struct device *dev;
	int num_sensors = ALIF_MAX_CSI_SENSORS;
	int nprocs = 0;
	int i;
	int ret;

	if (!sd)
		return -EINVAL;

	dev = sd->dev;
	start = &sd->entity;

	if (!fmt) {
		dev_err(dev, "invalid parameters passed\n");
		return -EINVAL;
	}
	requested = fmt->format;

	if (!start->graph_obj.mdev) {
		dev_err(dev, "Entity not part of media device\n");
		return -EINVAL;
	}

	ret = parse_camera_nodes(start, sensors, &num_sensors);
	if (ret)
		return ret;

	if (!num_sensors || !sensors[0]) {
		dev_err(dev, "No sensors found in the pipeline\n");
		return -ENODEV;
	}

	ret = media_graph_walk_init(&graph, start->graph_obj.mdev);
	if (ret) {
		dev_err(dev, "Failed to init graph walk: %d\n", ret);
		return ret;
	}

	media_graph_walk_start(&graph, &sensors[0]->entity);

	while ((entity = media_graph_walk_next(&graph))) {
		if (!is_media_entity_v4l2_subdev(entity))
			continue;
		if (entity == start || is_sensor_entity(entity))
			continue;
		if (nprocs >= ALIF_MAX_PIPELINE_PROCS) {
			media_graph_walk_cleanup(&graph);
			dev_err(dev, "Too many processing entities in pipeline (max %d)\n",
				ALIF_MAX_PIPELINE_PROCS);
			return -EINVAL;
		}
		procs[nprocs++] = entity;
	}
	media_graph_walk_cleanup(&graph);

	fmt->format = requested;
	ret = set_format_to_entity(&sensors[0]->entity, fmt, &requested,
				   state, dev);
	if (ret) {
		dev_err(dev, "Failed to set format for entity %s\n",
			sensors[0]->entity.name);
		return ret;
	}
	sensor_fmt = fmt->format;

	dev_dbg(dev, "Sensor %s accepted %ux%u code:0x%x\n",
		sensors[0]->entity.name, sensor_fmt.width, sensor_fmt.height,
		sensor_fmt.code);

	if (!nprocs)
		return 0;

	fmt->format = sensor_fmt;
	for (i = 0; i < nprocs; i++) {
		ret = set_format_to_entity(procs[i], fmt, &requested,
					   state, dev);
		if (ret) {
			dev_err(dev, "Failed to set format for entity %s\n",
				procs[i]->name);
			return ret;
		}

		dev_dbg(dev, "Set format on %s: %ux%u code:0x%x\n",
			procs[i]->name, fmt->format.width,
			fmt->format.height, fmt->format.code);
	}

	dev_dbg(dev, "Format propagation complete: %ux%u code:0x%x\n",
		fmt->format.width, fmt->format.height, fmt->format.code);

	return 0;
}
EXPORT_SYMBOL_GPL(pipeline_set_format);

MODULE_DESCRIPTION("V4L2 Pipeline Format Propagation Helper");
MODULE_AUTHOR("Yogender Kumar Arya <yogender.kumar@alifsemi.com>");
MODULE_LICENSE("GPL");
