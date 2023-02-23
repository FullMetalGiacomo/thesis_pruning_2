#include "dynamic_transform_publisher_alg.h"

DynamicTransformPublisherAlgorithm::DynamicTransformPublisherAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

DynamicTransformPublisherAlgorithm::~DynamicTransformPublisherAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void DynamicTransformPublisherAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// DynamicTransformPublisherAlgorithm Public API
